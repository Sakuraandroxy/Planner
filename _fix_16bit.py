# airsim_client.py: change get_depth_image_vlm to R+G base-256
fpath = r"E:\uni-lavira-code-main\sim\airsim_client.py"
c = open(fpath, "r", encoding="utf-8").read()
old1 = "    def get_depth_image_vlm(self):"
new1 = """
    def get_depth_image_vlm(self):
        """VLM depth: 16-bit value split across R+G channels (8-bit PNG).
        depth_cm = int(depth_meters * 100)
        R = (depth_cm >> 8) & 0xFF
        G = depth_cm & 0xFF
        B = 0
        Actual meters = (R*256 + G) / 100.
        Range: 0-655.35m, precision: 0.01m.
        Example: 300.25m -> depth_cm=30025 -> R=117,G=73 -> (117*256+73)/100.
        """
        responses = self.client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False)
        ])
        if not responses or not responses[0].image_data_float:
            return None
        r = responses[0]
        dm = np.array(r.image_data_float, dtype=np.float32).reshape(r.height, r.width)
        dm = np.clip(dm, 0, 655.35)
        depth_cm = (dm * 100).astype(np.uint32)
        R = ((depth_cm >> 8) & 0xFF).astype(np.uint8)
        G = (depth_cm & 0xFF).astype(np.uint8)
        B = np.zeros_like(R, dtype=np.uint8)
        rgb = np.stack([R, G, B], axis=-1)
        return Image.fromarray(rgb, mode="RGB")"""
idx = c.find(old1)
if idx >= 0:
    next_def = c.find("\n    def ", idx + len(old1))
    if next_def < 0: next_def = len(c)
    c = c[:idx] + new1 + c[next_def:]
    print("1. R+G base-256 (0-655.35m)")
else:
    print("1. NOT FOUND")
open(fpath, "w", encoding="utf-8").write(c)

# prompt_templates.py
fpath = r"E:\uni-lavira-code-main\planner\prompt_templates.py"
c = open(fpath, "r", encoding="utf-8").read()
c = c.replace("RGB编码：R=整数米，G=厘米*100。实际=R+G/100米，两位小数",
              "RGB编码：(R*256+G)/100=实际米数，两位小数。如R=117,G=73=300.25m")
c = c.replace("实际米数=R+G/100（两位小数）", "实际米数=(R*256+G)/100（两位小数）")
c = c.replace("实际距离=R+G/100（两位小数）", "实际距离=(R*256+G)/100（两位小数）")
open(fpath, "w", encoding="utf-8").write(c)
print("2. Prompts: (R*256+G)/100")

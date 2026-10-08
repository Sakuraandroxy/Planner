"""Image transport encoding shared by model adapters."""
import base64
import io


def image_url(image, image_format: str = "PNG") -> str:
    converted = image.convert("RGB") if image.mode != "RGB" else image
    buffer = io.BytesIO()
    converted.save(buffer, format=image_format, **({"quality": 90} if image_format == "JPEG" else {}))
    mime = "jpeg" if image_format == "JPEG" else "png"
    return f"data:image/{mime};base64,{base64.b64encode(buffer.getvalue()).decode('ascii')}"

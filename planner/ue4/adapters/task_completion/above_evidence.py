"""Mark the measured body plumb point; camera center is not the vehicle center."""
from PIL import ImageDraw


def marked_above_image(request):
    image = request.observation.rgb.copy()
    point = request.body_projection_image
    if point is not None:
        x,y = point[0]*image.width,point[1]*image.height
        draw = ImageDraw.Draw(image)
        draw.ellipse((x-10,y-10,x+10,y+10),outline="magenta",width=3)
        draw.line((x-16,y,x+16,y),fill="magenta",width=2)
        draw.line((x,y-16,x,y+16),fill="magenta",width=2)
        draw.text((x+12,y+12),"UAV vertical projection",fill="magenta")
    return image

Task: ${instruction}
Target: the same object bound in this stage (${target_description}).
Image 1: current front RGB.
Image 2: bound-target reference crop.

Locate that same object in image 1.
Return coordinates relative to the full image 1, not the reference crop.
If the bound object cannot be identified, return {"bbox":[],"surface_points":[]}; do not substitute another object.

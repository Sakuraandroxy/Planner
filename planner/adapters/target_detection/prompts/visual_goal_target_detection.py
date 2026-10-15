class VisualGoalTargetDetectionPrompt:
    def build(self, request):
        if request.reference_rgb is None:
            return (f"Task: {request.instruction}\nTarget: {request.target_description}.\n"
                    "Image 1: current front RGB. Return its target bbox.")
        return (f"Task: {request.instruction}\nTarget: the same previously bound target ({request.target_description}).\n"
                "Image 1: current front RGB.\nImage 2: bound-target reference crop.\n"
                "Return the same target's bbox in image 1 only.")

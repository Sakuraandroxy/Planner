import airsim, inspect
src = inspect.getsource(airsim.MultirotorClient.moveOnPathAsync)
print(src[:3000])

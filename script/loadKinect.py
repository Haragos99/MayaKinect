import maya.cmds as cmds


# For debug mode
path = r"C:\Users\Geri\Documents\Projects\CG\MayaKinect\out\build\x64-Debug\MayaKinect.mll"
# Load the plugin by name
if not cmds.pluginInfo(path, query=True, loaded=True):
    cmds.loadPlugin(path)
    
print("Plugin loaded and function executed successfully.")
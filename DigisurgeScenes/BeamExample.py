# Required import for python
import Sofa

from FingerController import FingerController

# Choose in your script to activate or not the GUI
USE_GUI = True


def main():
    import SofaRuntime
    import Sofa.Gui

    root = Sofa.Core.Node("root")
    createScene(root)
    Sofa.Simulation.init(root)

    if not USE_GUI:
        for iteration in range(10):
            Sofa.Simulation.animate(root, root.dt.value)
    else:
        Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
        Sofa.Gui.GUIManager.createGUI(root, __file__)
        Sofa.Gui.GUIManager.SetDimension(1080, 1080)
        Sofa.Gui.GUIManager.MainLoop(root)
        Sofa.Gui.GUIManager.closeGUI()
        
def createScene(root_node):

    root = root_node.addChild('root', gravity="0 -9.81 0", dt="0.01")#, time="0", animate="0", multiThreadSimulation="0")

    root.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Algorithm") 
    root.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Intersection") 
    root.addObject('RequiredPlugin', name="Sofa.Component.Collision.Geometry") 
    root.addObject('RequiredPlugin', name="Sofa.Component.Collision.Response.Contact") 
    root.addObject('RequiredPlugin', name="Sofa.Component.Constraint.Projective") 
    root.addObject('RequiredPlugin', name="Sofa.Component.LinearSolver.Iterative") 
    root.addObject('RequiredPlugin', name="Sofa.Component.Mapping.Linear") 
    root.addObject('RequiredPlugin', name="Sofa.Component.Mapping.NonLinear") 
    root.addObject('RequiredPlugin', name="Sofa.Component.Mass") 
    root.addObject('RequiredPlugin', name="Sofa.Component.ODESolver.Backward") 
    root.addObject('RequiredPlugin', name="Sofa.Component.SolidMechanics.Spring") 
    root.addObject('RequiredPlugin', name="Sofa.Component.StateContainer") 
    root.addObject('RequiredPlugin', name="Sofa.Component.Topology.Container.Constant") 
    root.addObject('RequiredPlugin', name="Sofa.Component.Visual") 
    root.addObject('RequiredPlugin', name="Sofa.GL.Component.Rendering3D") 
    root.addObject('VisualStyle', displayFlags="showBehaviorModels showCollisionModels") 
    root.addObject('BruteForceBroadPhase', ) 
    root.addObject('BVHNarrowPhase', ) 
    root.addObject('CollisionResponse', name="default1") 
    root.addObject('CollisionPipeline', name="default2") 
    root.addObject('MinProximityIntersection', name="default3", alarmDistance="1", contactDistance="0.5") 
    root.addObject('DefaultAnimationLoop', )

    default4 = root.addChild('default4', gravity="0 -9.81 0")

    default4.addObject('EulerImplicitSolver', name="cg_odesolver", printLog="0", rayleighStiffness="0.1", rayleighMass="0.1") 
    default4.addObject('CGLinearSolver', template="GraphScattered", name="linear solver", iterations="25", tolerance="1e-09", threshold="1e-09") 
    default4.addObject('MechanicalObject', template="Rigid3", name="DOFs", restScale="1", position="0 0 0 0 0 0 1 1 0 0 0 0 0 1 3 0 0 0 0 0 1 5 0 0 0 0 0 1 7 0 0 0 0 0 1", velocity="0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0") 
    default4.addObject('UniformMass', name="mass", vertexMass="1 1 [1 0 0,0 1 0,0 0 1]") 
    default4.addObject('FixedProjectiveConstraint', template="Rigid3", name="fixOrigin", indices="0")

    default5 = default4.addChild('default5', gravity="0 -9.81 0")

    default5.addObject('MechanicalObject', template="Rigid3", name="attaches", restScale="1", position="0 0 0 0.707914 0 0 0.707914 -1 0 0 0.707914 0 0 0.707914 1 0 0 0.707914 0 0 0.707914 -1 0 0 0.707914 0 0 0.707914 1 0 0 0.707914 0 0 0.707914 -1 0 0 0.707914 0 0 0.707914 1 0 0 0.707914 0 0 0.707914 -1 0 0 0.707914 0 0 0.707914 1 0 0 0.707914 0 0 0.707914", velocity="0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0") 
    default5.addObject('RigidMapping', template="Rigid3,Rigid3", name="default6", rigidIndexPerPoint="1 2 2 2 2", input="@../DOFs", output="@attaches") 
    default5.addObject('JointSpringForceField', template="Rigid3", name="joint springs", spring="BEGIN_SPRING 0 1 FREE_AXIS 0 0 0 0 1 0 KS_T 0 30000 KS_R 0 200000 KD 1 R_LIM_X -0.8 0.8 R_LIM_Y -1.57 1.57 R_LIM_Z 0 0 END_SPRING BEGIN_SPRING 2 3 FREE_AXIS 0 0 0 0 1 0 KS_T 0 30000 KS_R 0 200000 KD 1 R_LIM_X -0.8 0.8 R_LIM_Y -1.57 1.57 R_LIM_Z 0 0 END_SPRING BEGIN_SPRING 4 5 FREE_AXIS 0 0 0 0 1 0 KS_T 0 30000 KS_R 0 200000 KD 1 R_LIM_X -0.8 0.8 R_LIM_Y -1.57 1.57 R_LIM_Z 0 0 END_SPRING BEGIN_SPRING 6 7 FREE_AXIS 0 0 0 0 1 0 KS_T 0 30000 KS_R 0 200000 KD 1 R_LIM_X -0.8 0.8 R_LIM_Y -1.57 1.57 R_LIM_Z 0 0 END_SPRING ")

    default7 = default4.addChild('default7', gravity="0 -9.81 0")

    default7.addObject('MechanicalObject', template="Vec3", name="PointSet", restScale="1", position="0 -0.5 -0.5 0 0.5 -0.5 0 0.5 0.5 0 -0.5 0.5 2 -0.5 -0.5 2 0.5 -0.5 2 0.5 0.5 2 -0.5 0.5 2 -0.5 -0.5 2 0.5 -0.5 2 0.5 0.5 2 -0.5 0.5 4 -0.5 -0.5 4 0.5 -0.5 4 0.5 0.5 4 -0.5 0.5 4 -0.5 -0.5 4 0.5 -0.5 4 0.5 0.5 4 -0.5 0.5 6 -0.5 -0.5 6 0.5 -0.5 6 0.5 0.5 6 -0.5 0.5 6 -0.5 -0.5 6 0.5 -0.5 6 0.5 0.5 6 -0.5 0.5 8 -0.5 -0.5 8 0.5 -0.5 8 0.5 0.5 8 -0.5 0.5", velocity="0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0")
    default7.addObject('MeshTopology', name="default9", edges="0 1  1 2  2 3  3 0  1 5  5 4  4 0  5 6  6 7  7 4  2 6  7 3  8 9  9 10  10 11  11 8  9 13  13 12  12 8  13 14  14 15  15 12  10 14  15 11  16 17  17 18  18 19  19 16  17 21  21 20  20 16  21 22  22 23  23 20  18 22  23 19  24 25  25 26  26 27  27 24  25 29  29 28  28 24  29 30  30 31  31 28  26 30  31 27 ", triangles="3 1 0  3 2 1  3 6 2  3 7 6  7 5 6  7 4 5  4 1 5  4 0 1  5 1 2  2 6 5  4 7 3  4 3 0  11 9 8  11 10 9  11 14 10  11 15 14  15 13 14  15 12 13  12 9 13  12 8 9  13 9 10  10 14 13  12 15 11  12 11 8  19 17 16  19 18 17  19 22 18  19 23 22  23 21 22  23 20 21  20 17 21  20 16 17  21 17 18  18 22 21  20 23 19  20 19 16  27 25 24  27 26 25  27 30 26  27 31 30  31 29 30  31 28 29  28 25 29  28 24 25  29 25 26  26 30 29  28 31 27  28 27 24 ")
    default7.addObject('TriangleCollisionModel', template="Vec3", name="default10")
    default7.addObject('LineCollisionModel', name="default11")
    default7.addObject('SkinningMapping', template="Rigid3,Vec3", input="@../DOFs", output="@PointSet")

    visu = default4.addChild('Visu', gravity="0 -9.81 0")

    visu.addObject('OglModel', template="Vec3", name="Visual", position="0 -0.5 -0.5 0 0.5 -0.5 0 0.5 0.5 0 -0.5 0.5 2 -0.5 -0.5 2 0.5 -0.5 2 0.5 0.5 2 -0.5 0.5 2 -0.5 -0.5 2 0.5 -0.5 2 0.5 0.5 2 -0.5 0.5 4 -0.5 -0.5 4 0.5 -0.5 4 0.5 0.5 4 -0.5 0.5 4 -0.5 -0.5 4 0.5 -0.5 4 0.5 0.5 4 -0.5 0.5 6 -0.5 -0.5 6 0.5 -0.5 6 0.5 0.5 6 -0.5 0.5 6 -0.5 -0.5 6 0.5 -0.5 6 0.5 0.5 6 -0.5 0.5 8 -0.5 -0.5 8 0.5 -0.5 8 0.5 0.5 8 -0.5 0.5", triangles="3 1 0  3 2 1  3 6 2  3 7 6  7 5 6  7 4 5  4 1 5  4 0 1  5 1 2  2 6 5  4 7 3  4 3 0  11 9 8  11 10 9  11 14 10  11 15 14  15 13 14  15 12 13  12 9 13  12 8 9  13 9 10  10 14 13  12 15 11  12 11 8  19 17 16  19 18 17  19 22 18  19 23 22  23 21 22  23 20 21  20 17 21  20 16 17  21 17 18  18 22 21  20 23 19  20 19 16  27 25 24  27 26 25  27 30 26  27 31 30  31 29 30  31 28 29  28 25 29  28 24 25  29 25 26  26 30 29  28 31 27  28 27 24 ")
    visu.addObject('SkinningMapping', template="Rigid3,Vec3", input="@../DOFs", output="@Visual")

# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()
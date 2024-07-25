# Required import for python
import Sofa


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


def createScene(root):
    root.gravity=[0, -9.81, 0]
    root.dt=0.02

    root.addObject("RequiredPlugin", pluginName=[    'Sofa.Component.Collision.Detection.Algorithm',
    'Sofa.Component.Collision.Detection.Intersection',
    'Sofa.Component.Collision.Geometry',
    'Sofa.Component.Collision.Response.Contact',
    'Sofa.Component.Constraint.Projective',
    'Sofa.Component.IO.Mesh',
    'Sofa.Component.LinearSolver.Iterative',
    'Sofa.Component.Mapping.Linear',
    'Sofa.Component.Mass',
    'Sofa.Component.ODESolver.Backward',
    'Sofa.Component.SolidMechanics.Spring',
    'Sofa.Component.SolidMechanics.FEM.Elastic',
    'Sofa.Component.MechanicalLoad',
    'Sofa.Component.StateContainer',
    'Sofa.Component.Topology.Container.Dynamic',
    'Sofa.Component.Engine.Select',
    'Sofa.Component.Visual',
    'Sofa.GL.Component.Rendering3D'
    ])

    root.addObject('DefaultAnimationLoop')

    root.addObject('VisualStyle', displayFlags="hideCollisionModels")
    root.addObject('CollisionPipeline', name="CollisionPipeline")
    root.addObject('BruteForceBroadPhase', name="BroadPhase")
    root.addObject('BVHNarrowPhase', name="NarrowPhase")
    root.addObject('DefaultContactManager', name="CollisionResponse", response="PenalityContactForceField")
    root.addObject('DiscreteIntersection')

    root.addObject('MeshOBJLoader', name="MitralValveMesh", filename="mesh/mitral_valve_surface.obj")

    valve = root.addChild('Mitral_Valve')
    valve.addObject('EulerImplicitSolver', name="cg_odesolver", rayleighStiffness="0.005", rayleighMass="0.005")
    valve.addObject('CGLinearSolver', name="linear_solver", iterations="25", tolerance="1e-09", threshold="1e-09")
    valve.addObject('TriangleSetTopologyContainer', name="topo", src="@../MitralValveMesh")
    valve.addObject('MechanicalObject', name="dofs", src="@../MitralValveMesh")
    valve.addObject('TriangleSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo")
    valve.addObject('DiagonalMass', name="Mass", massDensity="1.0")
    valve.addObject('TriangularFEMForceFieldOptim', template="Vec3d", name="FEM", poissonRatio="0.3", youngModulus="300000")
    valve.addObject('FastTriangularBendingSprings', template="Vec3d", name="BendingFF", bendingStiffness="1000")
    valve.addObject('SphereROI', template="Vec3d", name="FixationROI", centers="0 0 0", radii="18", drawSphere="true")
    valve.addObject('RestShapeSpringsForceField', name="RestShapeSpringFF", points="@FixationROI.indicesOut", stiffness="1000")
    valve.addObject('SurfacePressureForceField', name="SurfacePressureFF", pressure="1000", pulseMode="true", pressureSpeed="10", mainDirection="0 1 0")
    valve.addObject('StiffSpringForceField', name="SpringPressureFF", stiffness="1000000", indices1="1582 1581 1580 1579 1578", indices2="802 803 764 765 766", lengths="0 0 0 0 0")

    visu = valve.addChild('Visu')
    visu.addObject('OglModel', name="VisualModel", src="@../../MitralValveMesh")
    visu.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@VisualModel")

    surf = valve.addChild('Surf')
    surf.addObject('MechanicalObject', name="spheres", position="@../../MitralValveMesh.position")
    surf.addObject('SphereCollisionModel', name="CollisionModel", radius="1.0")
    surf.addObject('BarycentricMapping', name="CollisionMapping", input="@../dofs", output="@spheres")

    return root


# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()

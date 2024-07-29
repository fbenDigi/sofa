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
    'Sofa.Component.SolidMechanics.FEM.Elastic',
    'Sofa.Component.StateContainer',
    'Sofa.Component.Topology.Container.Dynamic',
    'Sofa.Component.Engine.Select',
    'Sofa.Component.Visual',
    'Sofa.GL.Component.Rendering3D'
    ])

    root.addObject('DefaultAnimationLoop')

    root.addObject('VisualStyle', displayFlags="hideCollisionModels hideVisualModels showForceFields")
    root.addObject('CollisionPipeline', name="CollisionPipeline")
    root.addObject('BruteForceBroadPhase', name="BroadPhase")
    root.addObject('BVHNarrowPhase', name="NarrowPhase")
    root.addObject('DefaultContactManager', name="CollisionResponse", response="PenalityContactForceField")
    root.addObject('DiscreteIntersection')

    root.addObject('MeshOBJLoader', name="HeartSurface", filename="mesh/ValveMitral_Attached__sf.obj")

    heart = root.addChild('Heart')
    heart.addObject('EulerImplicitSolver', name="cg_odesolver", rayleighStiffness="0.1", rayleighMass="0.1")
    heart.addObject('CGLinearSolver', name="linear_solver", iterations="25", tolerance="1e-09", threshold="1e-09")
    heart.addObject('MeshGmshLoader', name="meshLoader", filename="mesh/ValveMitral_Attached.msh")
    heart.addObject('TetrahedronSetTopologyContainer', name="topo", src="@meshLoader")
    heart.addObject('MechanicalObject', name="dofs", src="@meshLoader")
    heart.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo")
    heart.addObject('DiagonalMass', name="Mass", massDensity="1.0")
    heart.addObject('TetrahedronFEMForceField', template="Vec3d", name="FEM", method="large", poissonRatio="0.3", youngModulus="3000", computeVonMisesStress="1", showVonMisesStressPerNodeColorMap="true")
    heart.addObject('BoxROI', template="Vec3d", name="FixationVentriculeBoxROI", box="-40.0 8.0 -40.0 40.0 15.0 40.0", drawBoxes="false")
    heart.addObject('BoxROI', template="Vec3d", name="FixationAtriumBoxROI", box="-40.0 -15.0 -40.0 40.0 -10.0 40.0", drawBoxes="false")
    heart.addObject('FixedConstraint', name="FixedConstraintVentricule", indices="@FixationVentriculeBoxROI.indices")
    heart.addObject('FixedConstraint', name="FixedConstraintAtrium", indices="@FixationAtriumBoxROI.indices")

    visu = heart.addChild('Visu')
    visu.addObject('OglModel', name="VisualModel", src="@../../HeartSurface")
    visu.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@VisualModel")

    surf = heart.addChild('Surf')
    # surf.addObject('SphereLoader', name="sphereLoader", filename="mesh/liver.sph")
    surf.addObject('MechanicalObject', name="spheres", position="@../../HeartSurface.position")
    surf.addObject('SphereCollisionModel', name="CollisionModel", radius="1.0")
    # surf.addObject('SphereCollisionModel', name="CollisionModel", listRadius="@sphereLoader.listRadius")
    surf.addObject('BarycentricMapping', name="CollisionMapping", input="@../dofs", output="@spheres")

    return root


# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()

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
    'Sofa.Component.SolidMechanics.Spring',
    'Sofa.Component.MechanicalLoad',
    'Sofa.Component.StateContainer',
    'Sofa.Component.Topology.Container.Dynamic',
    'Sofa.Component.Engine.Transform',
    'Sofa.Component.Engine.Select',
    'Sofa.Component.Visual',
    'Sofa.GL.Component.Rendering3D'
    ])

    root.addObject('DefaultAnimationLoop')

    root.addObject('VisualStyle', displayFlags="hideCollisionModels hideVisualModels showBehaviorModels showForceFields")
    root.addObject('CollisionPipeline', name="CollisionPipeline")
    root.addObject('BruteForceBroadPhase', name="BroadPhase")
    root.addObject('BVHNarrowPhase', name="NarrowPhase")
    root.addObject('CollisionResponse', name="CollisionResponseComp", response="PenalityContactForceField")
    root.addObject('DiscreteIntersection')

    root.addObject('MeshOBJLoader', name="HeartSurface", filename="mesh/Digisurge/GenerateMesh/Surface/ValveMitral_Attached__sf.obj")

    heart = root.addChild('Heart')
    heart.addObject('EulerImplicitSolver', name="cg_odesolver", rayleighStiffness="0.1", rayleighMass="0.1")
    heart.addObject('CGLinearSolver', name="linear_solver", iterations="25", tolerance="1e-09", threshold="1e-09")
    heart.addObject('MeshGmshLoader', name="meshLoader", filename="mesh/Digisurge/GenerateMesh/Volume/ValveMitral_Attached.msh")
    heart.addObject('TetrahedronSetTopologyContainer', name="topo", src="@meshLoader")
    heart.addObject('MechanicalObject', name="dofs", src="@meshLoader")
    heart.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo")
    heart.addObject('DiagonalMass', name="Mass", massDensity="1.0")
    heart.addObject('TetrahedronFEMForceField', template="Vec3d", name="FEM", method="large", poissonRatio="0.3", youngModulus="10000", computeVonMisesStress="1", showVonMisesStressPerNodeColorMap="true", showElementGapScale="0.0")
    heart.addObject('BoxROI', template="Vec3d", name="FixationVentriculeBoxROI", box="-40.0 8.0 -40.0 40.0 15.0 40.0", drawBoxes="false")
    heart.addObject('BoxROI', template="Vec3d", name="FixationAtriumBoxROI", box="-40.0 -15.0 -40.0 40.0 -10.0 40.0", drawBoxes="false")
    heart.addObject('FixedProjectiveConstraint', name="FixedConstraintVentricule", indices="@FixationVentriculeBoxROI.indices")
    heart.addObject('FixedProjectiveConstraint', name="FixedConstraintAtrium", indices="@FixationAtriumBoxROI.indices")
    heart.addObject('SphereROI', template="Vec3d", name="PressureROI", centers="-1 0 2", radii="24", drawSphere="false")
    heart.addObject('SurfacePressureForceField', name="SurfacePressureFF", pressure="16", pulseMode="true", pressureSpeed="10", mainDirection="0 1 0", triangleIndices="@PressureROI.triangleIndices")
    heart.addObject('BoxROI', template="Vec3d", name="Anterior2ROI", box="-10.0 -20.0 0.0 8.0 0.0 6.0", drawBoxes="false")
    heart.addObject('BoxROI', template="Vec3d", name="Posterior2ROI", box="-10.0 -20.0 8.0 8.0 0.0 14.0", drawBoxes="false")
    heart.addObject('SphereROI', template="Vec3d", name="Anterior1ROI", centers="-11 -8 -5 -9 -8 -1", radii="5 5", drawSphere="false")
    heart.addObject('SphereROI', template="Vec3d", name="Posterior1ROI", centers="-19 -5 2 -15 -5 10", radii="5 5", drawSphere="false")
    heart.addObject('SphereROI', template="Vec3d", name="Anterior3ROI", centers="11 -8 -5 9 -8 -1", radii="5 5", drawSphere="false")
    heart.addObject('SphereROI', template="Vec3d", name="Posterior3ROI", centers="16 -3 3 12 -3 10", radii="4 5", drawSphere="false")      
    heart.addObject('SphereROI', template="Vec3d", name="TrigonEastROI", centers="17 -1 -10", radii="2", drawSphere="false")      
    heart.addObject('SphereROI', template="Vec3d", name="TrigonWestROI", centers="-17 -5 -12", radii="2", drawSphere="false")      
    heart.addObject('SphereROI', template="Vec3d", name="AnchorP1ROI", centers="-22 -3 3", radii="2", drawSphere="true")      
    heart.addObject('SphereROI', template="Vec3d", name="AnchorP2ROI", centers="-3 0 20", radii="2", drawSphere="true")      
    heart.addObject('SphereROI', template="Vec3d", name="AnchorP3ROI", centers="19 0 3", radii="2", drawSphere="true")      
    

    chordesAnterior = heart.addChild('ChordesAnterior2')
    chordesAnterior.addObject('PointSetTopologyContainer', name="ChordesAnteriorTopo", position="@../Anterior2ROI.pointsInROI")
    chordesAnterior.addObject('TransformEngine', name="TEAnteriorChordes", input_position="@ChordesAnteriorTopo.position", translation="0 -50 -20", scale="0 0 0")
    chordesAnterior.addObject('PointSetGeometryAlgorithms')
    chordesAnterior.addObject('MechanicalObject', name="ChordesAnteriorCenter", position="@TEAnteriorChordes.output_position")
    chordesAnterior.addObject('BoxROI', template="Vec3d", name="AllPointsROI", box="-200 -200 -200 200 200 200", drawBoxes="false")
    chordesAnterior.addObject('FixedProjectiveConstraint', name="FixedChordesAnterior", indices="@AllPointsROI.indices")
    chordesAnterior.addObject('PolynomialSpringsForceField', name="ChordesAnteriorSpringFF", firstObjectPoints="@AllPointsROI.indices", secondObjectPoints="@../Anterior2ROI.indices", polynomialStiffness="2000", polynomialDegree="1", object1="@ChordesAnteriorCenter", object2="@../dofs", computeZeroLength="2", zeroLengthScale="1.0")

    chordesPosterior = heart.addChild('ChordesPosterior2')
    chordesPosterior.addObject('PointSetTopologyContainer', name="ChordesPosteriorTopo", position="@../Posterior2ROI.pointsInROI")
    chordesPosterior.addObject('TransformEngine', name="TEPosteriorChordes", input_position="@ChordesPosteriorTopo.position", translation="0 -50 20", scale="0 0 0")
    chordesPosterior.addObject('PointSetGeometryAlgorithms')    
    chordesPosterior.addObject('MechanicalObject', name="ChordesPosteriorCenter", position="@TEPosteriorChordes.output_position")
    chordesPosterior.addObject('BoxROI', template="Vec3d", name="AllPointsROI", box="-200 -200 -200 200 200 200", drawBoxes="false")
    chordesPosterior.addObject('FixedProjectiveConstraint', name="FixedChordesPosterior", indices="@AllPointsROI.indices")  
    chordesPosterior.addObject('PolynomialSpringsForceField', name="ChordesPosteriorSpringFF", firstObjectPoints="@AllPointsROI.indices", secondObjectPoints="@../Posterior2ROI.indices", polynomialStiffness="2000", polynomialDegree="1", object1="@ChordesPosteriorCenter", object2="@../dofs", computeZeroLength="2", zeroLengthScale="1.0")    


    chordesAnterior = heart.addChild('ChordesAnterior1')
    chordesAnterior.addObject('PointSetTopologyContainer', name="ChordesAnteriorTopo", position="@../Anterior1ROI.pointsInROI")
    chordesAnterior.addObject('TransformEngine', name="TEAnteriorChordes", input_position="@ChordesAnteriorTopo.position", translation="0 -50 -20", scale="0 0 0")
    chordesAnterior.addObject('PointSetGeometryAlgorithms')
    chordesAnterior.addObject('MechanicalObject', name="ChordesAnteriorCenter", position="@TEAnteriorChordes.output_position")
    chordesAnterior.addObject('BoxROI', template="Vec3d", name="AllPointsROI", box="-200 -200 -200 200 200 200", drawBoxes="false")
    chordesAnterior.addObject('FixedProjectiveConstraint', name="FixedChordesAnterior", indices="@AllPointsROI.indices")
    chordesAnterior.addObject('PolynomialSpringsForceField', name="ChordesAnteriorSpringFF", firstObjectPoints="@AllPointsROI.indices", secondObjectPoints="@../Anterior1ROI.indices", polynomialStiffness="2000", polynomialDegree="1", object1="@ChordesAnteriorCenter", object2="@../dofs", computeZeroLength="2", zeroLengthScale="1.0")

    chordesPosterior = heart.addChild('ChordesPosterior1')
    chordesPosterior.addObject('PointSetTopologyContainer', name="ChordesPosteriorTopo", position="@../Posterior1ROI.pointsInROI")
    chordesPosterior.addObject('TransformEngine', name="TEPosteriorChordes", input_position="@ChordesPosteriorTopo.position", translation="0 -50 20", scale="0 0 0")
    chordesPosterior.addObject('PointSetGeometryAlgorithms')    
    chordesPosterior.addObject('MechanicalObject', name="ChordesPosteriorCenter", position="@TEPosteriorChordes.output_position")
    chordesPosterior.addObject('BoxROI', template="Vec3d", name="AllPointsROI", box="-200 -200 -200 200 200 200", drawBoxes="false")
    chordesPosterior.addObject('FixedProjectiveConstraint', name="FixedChordesPosterior", indices="@AllPointsROI.indices")  
    chordesPosterior.addObject('PolynomialSpringsForceField', name="ChordesPosteriorSpringFF", firstObjectPoints="@AllPointsROI.indices", secondObjectPoints="@../Posterior1ROI.indices", polynomialStiffness="2000", polynomialDegree="1", object1="@ChordesPosteriorCenter", object2="@../dofs", computeZeroLength="2", zeroLengthScale="1.0")    


    chordesAnterior = heart.addChild('ChordesAnterior3')
    chordesAnterior.addObject('PointSetTopologyContainer', name="ChordesAnteriorTopo", position="@../Anterior3ROI.pointsInROI")
    chordesAnterior.addObject('TransformEngine', name="TEAnteriorChordes", input_position="@ChordesAnteriorTopo.position", translation="0 -50 -20", scale="0 0 0")
    chordesAnterior.addObject('PointSetGeometryAlgorithms')
    chordesAnterior.addObject('MechanicalObject', name="ChordesAnteriorCenter", position="@TEAnteriorChordes.output_position")
    chordesAnterior.addObject('BoxROI', template="Vec3d", name="AllPointsROI", box="-200 -200 -200 200 200 200", drawBoxes="false")
    chordesAnterior.addObject('FixedProjectiveConstraint', name="FixedChordesAnterior", indices="@AllPointsROI.indices")
    chordesAnterior.addObject('PolynomialSpringsForceField', name="ChordesAnteriorSpringFF", firstObjectPoints="@AllPointsROI.indices", secondObjectPoints="@../Anterior3ROI.indices", polynomialStiffness="2000", polynomialDegree="1", object1="@ChordesAnteriorCenter", object2="@../dofs", computeZeroLength="2", zeroLengthScale="1.0")

    chordesPosterior = heart.addChild('ChordesPosterior3')
    chordesPosterior.addObject('PointSetTopologyContainer', name="ChordesPosteriorTopo", position="@../Posterior3ROI.pointsInROI")
    chordesPosterior.addObject('TransformEngine', name="TEPosteriorChordes", input_position="@ChordesPosteriorTopo.position", translation="0 -50 20", scale="0 0 0")
    chordesPosterior.addObject('PointSetGeometryAlgorithms')    
    chordesPosterior.addObject('MechanicalObject', name="ChordesPosteriorCenter", position="@TEPosteriorChordes.output_position")
    chordesPosterior.addObject('BoxROI', template="Vec3d", name="AllPointsROI", box="-200 -200 -200 200 200 200", drawBoxes="false")
    chordesPosterior.addObject('FixedProjectiveConstraint', name="FixedChordesPosterior", indices="@AllPointsROI.indices")  
    chordesPosterior.addObject('PolynomialSpringsForceField', name="ChordesPosteriorSpringFF", firstObjectPoints="@AllPointsROI.indices", secondObjectPoints="@../Posterior3ROI.indices", polynomialStiffness="2000", polynomialDegree="1", object1="@ChordesPosteriorCenter", object2="@../dofs", computeZeroLength="2", zeroLengthScale="1.0")    


    chordesPosterior = heart.addChild('Implant') 
    # chordesPosterior.addObject('PolynomialSpringsForceField', name="ImplantTWP1SpringFF", firstObjectPoints="@../TrigonWestROI.indices", secondObjectPoints="@../AnchorP1ROI.indices", polynomialStiffness="1000000", polynomialDegree="1", object1="@../dofs", object2="@../dofs", computeZeroLength="2", zeroLengthScale="0.5")    
    # chordesPosterior.addObject('PolynomialSpringsForceField', name="ImplantP1P2SpringFF", firstObjectPoints="@../AnchorP1ROI.indices", secondObjectPoints="@../AnchorP2ROI.indices", polynomialStiffness="1000000", polynomialDegree="1", object1="@../dofs", object2="@../dofs", computeZeroLength="2", zeroLengthScale="0.5")    
    # chordesPosterior.addObject('PolynomialSpringsForceField', name="ImplantP2P3SpringFF", firstObjectPoints="@../AnchorP2ROI.indices", secondObjectPoints="@../AnchorP3ROI.indices", polynomialStiffness="1000000", polynomialDegree="1", object1="@../dofs", object2="@../dofs", computeZeroLength="2", zeroLengthScale="0.5")    
    # chordesPosterior.addObject('PolynomialSpringsForceField', name="ImplantP3TESpringFF", firstObjectPoints="@../AnchorP3ROI.indices", secondObjectPoints="@../TrigonEastROI.indices", polynomialStiffness="1000000", polynomialDegree="1", object1="@../dofs", object2="@../dofs", computeZeroLength="2", zeroLengthScale="0.5")    



    visu = heart.addChild('Visu')
    visu.addObject('OglModel', name="VisualModel", src="@../../HeartSurface")
    visu.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@VisualModel")

    surf = heart.addChild('Surf')
    surf.addObject('MechanicalObject', name="spheres", position="@../../HeartSurface.position")
    surf.addObject('SphereCollisionModel', name="CollisionModel", radius="1.0")
    surf.addObject('BarycentricMapping', name="CollisionMapping", input="@../dofs", output="@spheres")

    return root


# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()

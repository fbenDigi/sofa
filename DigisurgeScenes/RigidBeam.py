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

def createScene(root):
    root.gravity=[0, -9.81, 0]
    root.dt=0.02

    root.addObject("RequiredPlugin", pluginName=[ 'Sofa.Component.Constraint.Projective',   # Needed to use components [FixedProjectiveConstraint]
    'Sofa.Component.Engine.Generate',                                                       # Needed to use components [NormEngine]
    'Sofa.Component.Engine.Select',                                                         # Needed to use components [ValuesFromIndices]
    'Sofa.Component.Engine.Transform',                                                      # Needed to use components [DifferenceEngine]
    'Sofa.Component.LinearSolver.Direct',                                                   # Needed to use components [SparseLDLSolver]
    'Sofa.Component.LinearSolver.Iterative',                                                # Needed to use components [CGLinearSolver]
    'Sofa.Component.Mapping.Linear',                                                        # Needed to use components [BarycentricMapping]
    'Sofa.Component.Mapping.NonLinear',                                                     # Needed to use components [RigidMapping]
    'Sofa.Component.Mass',                                                                  # Needed to use components [MeshMatrixMass]
    'Sofa.Component.ODESolver.Backward',                                                    # Needed to use components [EulerImplicitSolver]
    'Sofa.Component.SolidMechanics.FEM.Elastic',                                            # Needed to use components [HexahedronFEMForceField]
    'Sofa.Component.SolidMechanics.Spring',
    'Sofa.Component.StateContainer',                                                        # Needed to use components [MechanicalObject]
    'Sofa.Component.Topology.Container.Dynamic',                                            # Needed to use components [HexahedronSetGeometryAlgorithms]
    'Sofa.Component.Topology.Container.Grid',                                               # Needed to use components [RegularGridTopology]
    'Sofa.Component.Visual',                                                                # Needed to use components [VisualStyle]
    'Sofa.GL.Component.Rendering2D',                                                        # Needed to use components [OglModel] 
    'Sofa.GL.Component.Rendering3D',                                                        # Needed to use components [OglModel] 
    'Sofa.Component.AnimationLoop',                                                         # Needed to use components [FreeMotionAnimationLoop]
    'Sofa.Component.Constraint.Lagrangian.Solver',                                          # Needed to use components [GenericConstraintSolver]
    'Sofa.Component.Constraint.Lagrangian.Correction',                                      # Needed to use components [GenericConstraintCorrection]
    'SoftRobots',                                                                           # Needed to use components [CableConstraint]

    ])
    
    root.addObject('FreeMotionAnimationLoop')
    root.addObject('DefaultVisualManagerLoop')
    root.addObject('VisualStyle',
                       displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')
    root.addObject('GenericConstraintSolver', tolerance=1e-5, maxIterations=100)

    catheter = root.addChild('catheter')
    catheter.addObject('EulerImplicitSolver', name="cg_odesolver", rayleighStiffness="0.25", rayleighMass="0.1")
    catheter.addObject('BTDLinearSolver', template="BTDMatrix6d", printLog="false", verbose="false")

    catheter.addObject('RegularGridTopology', name="grid", nx="1", ny="1", nz="30", xmin="0", xmax="0", ymin="-1.5", ymax="1.5", zmin="0", zmax="29")
    catheter.addObject('MechanicalObject', template="Rigid3", name="DOFs", position='@grid.position', showObject='true', showObjectScale='1.0')
    catheter.addObject('UniformMass', vertexMass='1 1 [1 0 0,0 1 0,0 0 1]')
    catheter.addObject('BeamFEMForceField', name="FEM", radius="5", radiusInner="4", youngModulus="1000000", poissonRatio="0.3")
    # catheter.addObject('JointSpringForceField', template="Rigid3", name="joint springs", spring="BEGIN_SPRING 0 1 FREE_AXIS 0 0 0 0 1 0 KS_T 0 10 KS_R 0 10 KD 1 R_LIM_X -0.8 0.8 R_LIM_Y -1.57 1.57 R_LIM_Z 0 0 REST_T 0 0 30 END_SPRING")

    catheter.addObject('FixedProjectiveConstraint', name="FixedCatheterSegment", indices="0")
    catheter.addObject('GenericConstraintCorrection')

    shape = catheter.addChild('shape')
    shape.addObject('CylinderGridTopology', name="grid", nx="5", ny="5", nz="10", radius="5.1", length="30")
    shape.addObject('MechanicalObject', name="shapeMO", position="@grid.position")
    shape.addObject('EdgeSetGeometryAlgorithms', drawEdges="true")
    shape.addObject('BoxROI', template="Vec3d", name="cableRow1", box="-0.5 4.5 -0.5 0.5 5.5 31", drawBoxes="false")
    #TODO : Grid
    shape.addObject('CableConstraint', name="aCableActuator", indices='22 47 72 97 122 147 172 197 222 247', minForce=0,  # Set that the cable can't push
                        pullPoint="0 5 -10") #@topo.indices
    shape.addObject(FingerController(name="FingerController", node=shape))
    shape.addObject('SkinningMapping', template="Rigid3,Vec3", input="@../DOFs", output="@shapeMO")

    return root

# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()
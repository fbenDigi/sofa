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
    # root.gravity=[0, -9.81, 0]
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
    catheter.addObject('EulerImplicitSolver', name="cg_odesolver", rayleighStiffness="0.1", rayleighMass="0.1")
    catheter.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixMat3x3d')

    catheter.addObject(
        Cosserat(parent=solverNode, cosseratGeometry=needleGeometryConfig, radius=GeometryParams.radius,
                 name="needle", youngModulus=PhysicsParams.youngModulus, poissonRatio=PhysicsParams.poissonRatio,
                 rayleighStiffness=PhysicsParams.rayleighStiffness))

    catheter.addObject('FixedProjectiveConstraint', name="FixedCatheterSegment", indices="0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24")
    catheter.addObject('GenericConstraintCorrection')

    actuator = catheter.addChild('actuator') 
    actuator.addObject('MechanicalObject', position=[[0, 5, 0], [0, 4, 0],
                                                     [0, 5, 3], [0, 4, 3],
                                                     [0, 5, 6], [0, 4, 6],
                                                     [0, 5, 9], [0, 4, 9],
                                                     [0, 5, 12], [0, 4, 12],
                                                     [0, 5, 15], [0, 4, 15],
                                                     [0, 5, 18], [0, 4, 18],
                                                     [0, 5, 21], [0, 4, 21],
                                                     [0, 5, 24], [0, 4, 24],
                                                     [0, 5, 27], [0, 4, 27],
                                                     [0, 5, 30], [0, 4, 30]]) 
    actuator.addObject('CableConstraint', name="aCableActuator", indices='0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19', minForce=0,  # Set that the cable can't push
                        pullPoint="0 5 -10") 
    actuator.addObject('BarycentricMapping')
    actuator.addObject(FingerController(name="FingerController", node=actuator))

    visual = catheter.addChild("Visual")
    visual.addObject('CylinderGridTopology', name="grid", nx="5", ny="5", nz="30", radius="5", length="30", computeTriangleList="false")
    visual.addObject('OglModel', name="visu", lineWidth="5", material="Default Diffuse 0 1 1 1 1 Ambient 1 0 1 0 1 Specular 0 1 0 0 1 Emissive 0 1 0 0 1 Shininess 0 45")
    visual.addObject('BarycentricMapping', input="@../DOFs", output="@visu")
    return root

# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()
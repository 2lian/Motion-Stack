import omni.graph.core as og


def setup(db: og.Database):
    state = db.internal_state
    state.allowed_joint_names = set()


def cleanup(db: og.Database):
    pass


def compute(db: og.Database):
    """
    Filter out the joints that are not part of the robot
    """
    state = db.internal_state

    # Accumulate all the joint names
    state.allowed_joint_names.update(db.inputs.allowed_joint_names)

    # Filter input
    mask = [name in state.allowed_joint_names for name in db.inputs.jointNames]
    db.outputs.jointNames = [name for name, m in zip(db.inputs.jointNames, mask) if m]
    db.outputs.positionCommand = [
        position for position, m in zip(db.inputs.positionCommand, mask) if m
    ]
    db.outputs.velocityCommand = [
        velocity for velocity, m in zip(db.inputs.velocityCommand, mask) if m
    ]
    db.outputs.effortCommand = [
        effort for effort, m in zip(db.inputs.effortCommand, mask) if m
    ]
    return True

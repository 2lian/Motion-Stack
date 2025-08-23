def patch():
    """Applies the patch to rtb"""
    import roboticstoolbox.tools.urdf.urdf as bad

    import fixed_urdf as fix

    bad.URDF.__init__ = fix.URDF.__init__
    bad.URDF._recursive_axis_definition = fix.URDF._recursive_axis_definition
    bad.URDF.finalize_linking = fix.URDF.finalize_linking

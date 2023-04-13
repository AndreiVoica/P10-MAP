

def _change_property(self, prim_path: str, attribute_name:str, value:float):
    usd_path = Sdf.Path(prim_path + "." + attribute_name)
    omni.kit.commands.execute(
        "ChangeProperty",
        prop_path=usd_path,
        value=value,
        prev=self._get_property(prim_path, attribute_name),
    )

def _get_property(self, prim_path: str, attribute: str):
    prim= self.stage.GetPrimAtPath(prim_path)
    prim_property = prim.GetAttribute(attribute)
    return prim_property.Get()
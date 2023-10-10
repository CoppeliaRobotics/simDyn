local codeEditorInfos = [[
int injectionId = simMujoco.composite(string xml, map info)
map info = simMujoco.getCompositeInfo(int injectionId, int what)
int injectionId = simMujoco.injectXML(string xml, string element, map info)
simMujoco.removeXML(int injectionId)
]]
registerCodeEditorInfos("simMujoco", codeEditorInfos)

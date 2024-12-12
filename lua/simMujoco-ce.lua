local codeEditorInfos = [[
int injectionId = simMujoco.addFlexcomp(map info)
int injectionId = simMujoco.composite(string xml, map info)
map info = simMujoco.getFlexcompInfo(int injectionId, int what)
map info = simMujoco.getCompositeInfo(int injectionId, int what)
string info = simMujoco.getInfo(string what)
int injectionId = simMujoco.addInjection(map info)
simMujoco.removeInjection(int injectionId)
]]
registerCodeEditorInfos("simMujoco", codeEditorInfos)

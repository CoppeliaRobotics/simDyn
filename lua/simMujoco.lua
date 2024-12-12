local simMujoco = loadPlugin 'simMujoco';

function simMujoco.addFlexcomp(...)
    local info = checkargs({{type = 'table'}}, ...)
    local lb = sim.setStepping(true)
    function __cb1(flexcompId)
        local r = _S.mujocoCbs[info.name](flexcompId)
        return r
    end
    local funcNm, t
    if info.callback then
        if not _S.mujocoCbs then _S.mujocoCbs = {} end
        _S.mujocoCbs[info.name] = info.callback
        info.cbFunc = '__cb1'
    end
    local injectionId = simMujoco._addFlexcomp(info)
    sim.setStepping(lb)
    return injectionId
end

function simMujoco.addInjection(...)
    local info = checkargs({{type = 'table'}}, ...)
    local lb = sim.setStepping(true)
    function __cb1(flexcompId)
        local r = _S.mujocoCbs[info.name](flexcompId)
        return r
    end
    local funcNm, t
    if info.callback then
        if not _S.mujocoCbs then _S.mujocoCbs = {} end
        _S.mujocoCbs[info.name] = info.callback
        info.cbFunc = '__cb1'
    end
    local injectionId = simMujoco._addInjection(info)
    sim.setStepping(lb)
    return injectionId
end

function simMujoco.removeInjection(...)
    return simMujoco._removeInjection(...)
end

function simMujoco.getFlexcompInfo(...)
    return simMujoco._getFlexcompInfo(...)
end

function simMujoco.getInfo(...)
    return simMujoco._getInfo(...)
end

function simMujoco.composite(...)
    local xml, info = checkargs({{type = 'string'}, {type = 'table'}}, ...)
    local lb = sim.setStepping(true)
    function __cb1(xml, info)
        local xmlOut, infoOut = _S.mujocoCbs[info.prefix](xml, info)
        return xmlOut, infoOut
    end
    local funcNm, t
    if info.callback then
        if not _S.mujocoCbs then _S.mujocoCbs = {} end
        _S.mujocoCbs[info.prefix] = info.callback
        info.cbFunc = '__cb1'
    end
    local injectionId = simMujoco._composite(xml, info)
    sim.setStepping(lb)
    return injectionId
end

function simMujoco.injectXML(...)
    local xml, item2, info = checkargs({
        {type = 'string'}, {type = 'any'}, {type = 'table', default = NIL, nullable = true},
    }, ...)
    local lb = sim.setStepping(true)
    function __cb2(xml, info)
        local xmlOut = _S.mujocoCbs[info.cbId](xml)
        return xmlOut
    end
    local funcNm, t
    info = info or {}
    if info.callback then
        if not _S.mujocoCbs then _S.mujocoCbs = {} end
        local id = 0
        while _S.mujocoCbs['+' .. id] do id = id + 1 end
        id = '+' .. id
        _S.mujocoCbs[id] = info.callback
        info.cbFunc = '__cb2'
        info.cbId = id
    end
    local injectionId = simMujoco._addInjection(xml, item2, info)
    sim.setStepping(lb)
    return injectionId
end

function simMujoco.removeXML(...)
    return simMujoco.removeInjection(...)
end

function simMujoco.getCompositeInfo(...)
    return simMujoco._getCompositeInfo(...)
end

return simMujoco

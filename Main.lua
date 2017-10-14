
supportedOrientations(LANDSCAPE_ANY)
function setup()
    displayMode(OVERLAY)
    displayMode(FULLSCREEN)

    extendModel()
    extendQuat()
    g = .0000981
    trackFunction, maxHeight = Tracks.loop(3,2)
    scene = craft.scene()

    trolly = scene:entity()
    cam = scene:entity()
    cam:add(craft.camera)
    cam.parent = trolly
    cam.position = vec3(0,2,-5)
    
    ball = scene:entity()
    ball.model = craft.model.sphere({radius = .5, axes = {vec3(0,1,0),vec3(0,0,1),vec3(1,0,0)}})
    ball.material = craft.material.preset("Surfaces:Basic Bricks")
    ball.parent = trolly
    ball.position = vec3(0,.25,0)

    track, rotfn = TrackPoints({
        pathFunction = trackFunction,
        delta = .01,
        step = .3,
        closed = true,
        twist = 5
    })
    local e = scene:entity()
    local planks = PseudoMesh()
    for k,v in ipairs(track) do
        planks:addBlock({
            centre = v[1],
            width = vec3(2,0,0)^v[2],
            height = .1*vec3(0,1,0)^v[2],
            depth = .2*vec3(0,0,1)^v[2],
        })
    end
    e.model = planks:toModel()
    e.material = craft.material.preset("Surfaces:Desert Cliff")
    size = .1
    local stars = PseudoMesh()
    for i=1,200 do
        addStar(stars,{
            origin = vec3(
            40*(2*math.random()-1),
            10*(2*math.random()-1),
            40*(2*math.random()-1)
            ),
            size = .25
        })

        addStar(stars,{
            origin = 100*RandomVec3(),
            size = .25
        })
    end
    e = scene:entity()
    e.model = stars:toModel()
    e.material = craft.material("Materials:Specular")
    local galaxy = PseudoMesh()
    galaxy:addSphere({radius = 100})
    galaxy:invertNormals()
    e = scene:entity()
    e.model = galaxy:toModel()
    e.material = craft.material("Materials:Basic")
    e.material.map = "Dropbox:starmap_small"
    time = 0
    speed = .002
    ht = .5

    energy = speed^2 + 2*g*(maxHeight +5)

    paused = true
    ball.active = false
    ptext = "Tap to begin"
    brot = quat.angleAxis(9,vec3(1,0,0))
end

function draw()
    scene:update(DeltaTime)
    background(0,0,0)
    if not paused then
        time = time + speed*DeltaTime
        if time > 1 and isRecording() then
            stopRecording()
        end
   
        local pos = trackFunction(time)
        local rot = rotfn(time)
        speed = math.sqrt(energy - 2*g*pos.y)

        trolly.position = pos + ht*vec3(0,1,0)^rot
        trolly.rotation = rot
        ball.rotation = quat.angleAxis(time*6000,vec3(1,0,0))
        cam.rotation = brot
    else
        trolly.position = vec3(0,8*maxHeight,15)
    end
    scene:draw()
    if paused then
        font("Copperplate-Bold")
        fontSize(30)
        fill(255, 0, 221, 255)
        text(ptext,WIDTH/2,HEIGHT/2)
    end

end

function touched(touch)
    if paused then
        paused = false
        ball.active = true
        -- startRecording()
        return
    end
    if touch.state == ENDED and touch.tapCount == 2 then
        paused = true
        ball.active = false
        ptext = "Tap to resume"
        return
    end
    brot = brot * quat.tangent(touch.deltaY/HEIGHT*math.pi,touch.deltaX/WIDTH*math.pi,0) 
end

function TrackPoints(a)
    a = a or {}
    local pts = a.points or {}
    local t = a.start or 0
    local r = a.step or .1
    local s = a.delta or .1
    local f = a.pathFunction or function(q) return q*vec3(1,0,0) end
    local b = a.finish or 1
    local tpt = f(t)
    local tgt = tangent({delta = s, pathFunction = f, time = t}):normalise()
    local iq = vec3(0,0,1):rotateTo(tgt)
    table.insert(pts,{tpt,iq,t})
    local dis
    local p,tg,q
    q = iq
    while t < b do
        dis = 0
        while dis < r do
            t = t + s
            p = f(t)
            dis = dis + p:dist(tpt)
            tpt = p
        end
        if t > b then
            t = b
            p = f(b)
        end
        tg = tangent({delta = s, pathFunction = f, time = t}):normalise()
        q = tgt:rotateTo(tg)*q
        table.insert(pts,{p,q,t})
        tpt = p
        tgt = tg
    end
    local st = a.start or 0
    local sc = b - st
    if a.closed then
        local twt = a.twist or 0
        twt = twt * 360
        local tw = function(r) return quat.angleAxis(r*twt,vec3(0,0,1)) end
        local hol = quat(1,0,0,0):make_slerp(q^""*iq)
        for k,v in ipairs(pts) do
            v[2] = v[2]*hol((v[3]-st)/sc)*tw((v[3]-st)/sc)
        end
    end
    local rotfn = function(t)
        t = t - math.floor(t)
        local sk = 0
        local sv = pts[1]
        for k,v in ipairs(pts) do
            if t < v[3] then
                break
            end
            sk = k
            sv = v
        end
        local ev,et
        if pts[sk+1] then
            ev = pts[sk+1][2]
            et = pts[sk+1][3]
        else
            ev = pts[0][2]
            et = 1
        end
        return sv[2]:slerp(ev, (t-sv[3])/(et-sv[3]))
    end
    return pts,rotfn
end

local starColours = {
    color(255,249,205,255),
    color(237,232,191,255),
    color(205,201,165,255),
    color(138,136,112,255)
}

function addStar(m,t)
    local o = t.origin
    local s = t.size
    local b = RandomBasisR3()
    local a = {
        s*math.sqrt(2)*b[1],
        s*b[2],
        s*b[3],
    }
    local as = 1/math.sqrt(2)
    local c = starColours[math.random(1,4)]

    m:addPyramid({
        origin = o - a[1]/4,
        axes = a,
        aspect = as,
        sides = 3,
        colour = c
    })
    for k=1,3 do
        a[k] = -a[k]
    end

    m:addPyramid({
        origin = o-a[1]/4,
        axes = a,
        aspect = as,
        sides = 3,
        colour = c
    })
end

function tangent(a)
    local s = a.delta/2 or .1
    local f = a.pathFunction or function(q) return q*vec3(1,0,0) end
    local t = a.time or 0
    local u = f(t-s)
    local v = f(t+s)
    return (v-u)/(2*s)
end

Tracks = {}

function Tracks.torus(p,q)
    local innerRa = 10
    local innerRb = 10
    local outerR = 30
    local trackFunction = function(t)
        local it = p*t*2*math.pi
        local ot = q*t*2*math.pi
            return vec3(
                    (outerR + innerRb*math.cos(it))*math.cos(ot),
                    innerRa*math.sin(it),
                    (outerR + innerRb*math.cos(it))*math.sin(ot)
                    )
            end
    local coreFunction = function(t)
        local ot = q*t*2*math.pi
            return vec3(
                    outerR*math.cos(ot),
                    0,
                    outerR*math.sin(ot)
                    )
            end
    local maxHeight = innerRa
    return trackFunction, maxHeight
end

function Tracks.mobius()
    local r = 30
    local trackFunction = function(t)
            local a = 2*math.pi*t
            return vec3(r*math.cos(a),0,r*math.sin(a))
        end
    return trackFunction,0
end

function Tracks.loop(p,q)
    local r = 30
    local h = 10
    local w = vec3(0,30,0)
    local trackFunction = function(t)
            local a = 2*math.pi*t
            return vec3(
            r*math.cos(a),
            h*math.sin(p*a),
            r*math.sin(q*a))
        end
    return trackFunction,h
end

function RandomVec3()
    local th = 2*math.pi*math.random()
    local z = 2*math.random() - 1
    local r = math.sqrt(1 - z*z)
    return vec3(r*math.cos(th),r*math.sin(th),z)
end

function RandomBasisR3()
    local th = 2*math.pi*math.random()
    local cth = math.cos(th)
    local sth = math.sin(th)
    local a = vec3(cth,sth,0)
    local b = vec3(-sth,cth,0)
    local c = vec3(0,0,1)
    local v = RandomVec3()
    a = a - 2*v:dot(a)*v
    b = b - 2*v:dot(b)*v
    c = c - 2*v:dot(c)*v
    return {a,b,c}
end


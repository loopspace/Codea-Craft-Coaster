-- Extensions to the native Codea vector and matrix types
-- Author: Andrew Stacey
-- Website: http://loopspace.mathforge.com
-- Licence: CC0 http://wiki.creativecommons.org/CC0

--[[
vec4s are promoted to quaternions, vec2s to complex numbers, and other functions are adapted to make use of them
--]]

-- Simplistic error handling

local error = error or print

-- Localise the maths functions/constants that we use for faster lookup.
local abs = math.abs
local pow = math.pow
local sqrt = math.sqrt
local sin = math.sin
local cos = math.cos
local acos = math.acos
local asin = math.asin
local pi = math.pi
local floor = math.floor
local min = math.min
local max = math.max
local exp = math.exp
local log = math.log
local tan = math.tan
local sinh = math.sinh
local cosh = math.cosh
local tanh = math.tanh
local huge = math.huge

local tolerance = 0.0000001

local m,mq

--[[
The function "is_a" extends the capabilities of the method "is_a" which is automatically defined by Codea for classes.

Parameters:
a: object to be tested
b: test

The tests work as follows.

1. If the type of b is a string, it is taken as the name of a type to test a against.
2. If the type of b is a table, it is assumed to be a class to test if a is an instance thereof.
3. If the type of b is a userdata, the test is to see if a is the same type of object.
4. If b is a function, then it is replaced by the value of that function.
--]]

function is_a(a,b)
    if type(b) == "function" then
        b = b()
    end
    if type(b) == "table" and b.___type then
        b = b()
    end
    if type(b) == "string" then
        return type(a) == b
    end
    if type(b) == "table"
    and type(a) == "table"
    and a.is_a
    then
        return a:is_a(b)
    end
    if type(b) == "userdata"
    and type(a) == "userdata"
    then
        if a.___type or b.___type then
            return a.___type == b.___type
        end
        return  getmetatable(a) == getmetatable(b)
    end
    return false
end

-- Using the new quat class

function extendQuat()
    local mq,m
    mq = getmetatable(quat())
    if mq.__extended then
        return
    end
    
    local function __quat(a,b,c,d)
        local qq = quat()
        q.w = a
        q.x = b
        q.y = c
        q.z = d
        return q
    end
    m = {}
    m["is_finite"] = function(q)
        if q.x < huge
        and q.x > -huge
        and q.y < huge
        and q.y > -huge
        and q.z < huge
        and q.z > -huge
        and q.w < huge
        and q.w > -huge
        then
            return true
        end
        return false
    end
    
    m["is_real"] = function (q)
        if q.y ~= 0
        or q.z ~= 0
        or q.x ~= 0
        then
            return false
        end
        return true
    end
    
    m["is_imaginary"] = function (q)
        return q.w == 0
    end
    
    m["normalise"] = function (q)
        q = q:normalize()
        if q:is_finite() then
            return q
        else
            return quat(1,0,0,0)
        end
    end
    
    m["len"] = function(q)
        return sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
    end
    
    m["lenSqr"] = function(q)
        return q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
    end
    
    m["dist"] = function(q,qq)
        return sqrt((q.x-qq.x)^2+(q.y-qq.y)^2+(q.z-qq.z)^2+(q.w-qq.w)^2)
    end
    
    m["distSqr"] = function(q,qq)
        return (q.x-qq.x)^2+(q.y-qq.y)^2+(q.z-qq.z)^2+(q.w-qq.w)^2
    end
    
    m["dot"] = function(q,qq)
        return q.x*qq.x + q.y*qq.y + q.z*qq.z + q.w*qq.w
    end
    
    m["normalize"] = function(q)
        return q/q:len()
    end
    
    m["slen"] = function(q)
        q = q:normalise()
        q.w = q.w - 1
        return 2*asin(q:len()/2)
    end
    
    m["sdist"] = function(q,qq)
        q = q:normalise()
        qq = qq:normalise()
        return 2*asin(q:dist(qq)/2)
    end
    
    m["len1"] = function(c)
        return abs(c.x) + abs(c.y) + abs(c.z) + abs(c.w)
    end
    
    m["dist1"] = function(c,v)
        return abs(c.x - v.x) + abs(c.y - v.y) + abs(c.z - v.z) + abs(c.w - v.w)
    end
    
    m["leninf"] = function(c)
        return max(abs(c.x), abs(c.y), abs(c.z), abs(c.w))
    end
    
    m["distinf"] = function(c,v)
        return max(abs(c.x - v.x), abs(c.y - v.y), abs(c.z - v.z), abs(c.w - v.w))
    end
    
    local mulq = mq["__mul"]
    
    rawset(quat,"tangent",function(x,y,z,t)
        local q
        if is_a(x,"number") then
            q,t = quat(0,x,y,z), t or 1
        else
            q,t = quat(0,x.x,x.y,x.z), y or 1
        end
        local qn = q:normalise()
        if qn == quat(1,0,0,0) then
            return qn
        end
        t = t * q:len()
        return cos(t)*quat(1,0,0,0) + sin(t)*qn
    end)
    
    m["__add"] = function (a,b)
        if is_a(a,"number") then
            a = quat(a,0,0,0)
        end
        if is_a(b,"number") then
            b = quat(b,0,0,0)
        end
        return quat(a.w+b.w,a.x+b.x,a.y+b.y,a.z+b.z)
    end
    
    m["__sub"] = function (a,b)
        if is_a(a,"number") then
            a = quat(a,0,0,0)
        end
        if is_a(b,"number") then
            b = quat(b,0,0,0)
        end
        return quat(a.w-b.w,a.x-b.x,a.y-b.y,a.z-b.z)
    end
    
    m["__mul"] = function (a,b)
        if is_a(a,"number") then
            return quat(a*b.w,a*b.x,a*b.y,a*b.z)
        end
        if is_a(b,"number") then
            return quat(a.w*b,a.x*b,a.y*b,a.z*b)
        end
        if is_a(a,matrix) then
            return a:__mul(b:tomatrixleft())
        end
        if is_a(b,matrix) then
            return a:tomatrixleft():__mul(b)
        end
        return mulq(a,b)
    end
    
    m["conjugate"] = function (q)
        return quat(q.w,-q.x,-q.y,-q.z)
    end
    
    m["co"] = m["conjugate"]
    
    m["__div"] = function (a,b)
        if is_a(b,"number") then
            return quat(a.w/b,a.x/b,a.y/b,a.z/b)
        end
        local l = b:lenSqr()
        b = quat(b.w/l,-b.x/l,-b.y/l,-b.z/l)
        if is_a(a,"number") then
            return quat(a*b.w,a*b.x,a*b.y,a*b.z)
        end
        return mulq(a,b)
    end
    
    function integerpower(q,n)
        if n == 0 then
            return quat(1,0,0,0)
        elseif n > 0 then
            return q:__mul(integerpower(q,n-1))
        elseif n < 0 then
            local l = q:lenSqr()
            q = quat(q.w/l,-q.x/l,-q.y/l,-q.z/l)
            return integerpower(q,-n)
        end
    end
    
    function realpower(q,n)
        if n == floor(n) then
            return integerpower(q,n)
        end
        local l = q:len()
        q = q:normalise()
        return l^n * q:slerp(n)
    end
    
    m["__pow"] = function (q,n)
        if is_a(n,"number") then
            return realpower(q,n)
        elseif is_a(n,quat) then
            return n:__mul(q):__div(n)
        else
            return q:conjugate()
        end
    end
    
    m["lerp"] = function (q,qq,t)
        if not t then
            q,qq,t = quat(1,0,0,0),q,qq
        end
        if (q + qq):len() == 0 then
            q = (1 - 2*t) * q + (1 - abs(2*t - 1)) * quat(q.x,-q.w,q.z,-q.y)
        else
            q = (1-t)*q + t*qq
        end
        return q:normalise()
    end
    --[[
    m["slerp"] = function (q,qq,t)
        if not t then
            q,qq,t = quat(1,0,0,0),q,qq
        end
        if (q + qq):len() == 0 then
            qq,t = quat(q.x,-q.w,q.z,-q.y),2*t
        elseif (q - qq):len() == 0 then
            return q
        end
        local ca = q:dot(qq)
        local sa = sqrt(1 - pow(ca,2))
        if sa == 0 or sa ~= sa then
            return q
        end
        local a = acos(ca)
        sa = sin(a*t)/sa
        return (cos(a*t)-ca*sa)*q+sa*qq
    end
    --]]
    m["make_lerp"] = function (q,qq)
        if not qq then
            q,qq = quat(1,0,0,0),q
        end
        q,qq = q:normalise(),qq:normalise()
        if (q + qq):len() == 0 then
            qq = quat(q.x,-q.w,q.z,-q.y)
            return function(t)
                return ((1-2*t)*q+(1-abs(2*t-1))*qq):normalise()
            end
        else
            return function(t)
                return ((1-t)*q+t*qq):normalise()
            end
            
        end
    end
    
    m["make_slerp"] = function (q,qq)
        if not qq then
            q,qq = quat(1,0,0,0),q
        end
        q,qq = q:normalise(),qq:normalise()
        local f
        if (q + qq):len() == 0 then
            qq,f = quat(q.x,-q.w,q.z,-q.y),2
        elseif (q - qq):len() == 0 then
            return function(t)
                return q
            end
        else
            f = 1
        end
        local ca = q:dot(qq)
        local sa = sqrt(1 - pow(ca,2))
        if sa == 0 or sa ~= sa then
            return function(t)
                return q
            end
        end
        local a = acos(ca)
        qq = (qq - ca*q)/sa
        return function(t)
            return cos(a*f*t)*q + sin(a*f*t)*qq
        end
    end
    
    m["toreal"] = function (q)
        return q.w
    end
    
    m["vector"] = function (q)
        return vec3(q.x, q.y, q.z)
    end
    
    m["tovector"] = m["vector"]
    
    m["log"] = function (q)
        local l = q:slen()
        q = q:tovector():normalize()
        if not q:is_finite() then
            return vec3(0,0,0)
        else
            return q * l
        end
    end
    
    m["tostring"] = function (q)
        local s
        local im = {{q.x,"i"},{q.y,"j"},{q.z,"k"}}
        if q.x ~= 0 then
            s = string.format("%.3f",q.w)
        end
        for k,v in pairs(im) do
            if v[1] ~= 0 then
                if s then
                    if v[1] > 0 then
                        if v[1] == 1 then
                            s = s.." + "..v[2]
                        else
                            s = s.." + "..string.format("%.3f",v[1])..v[2]
                            
                        end
                    else
                        if v[1] == -1 then
                            s = s.." - "..v[2]
                        else
                            s = s.." - "..string.format("%.3f",-v[1])..v[2]
                        end
                    end
                else
                    if v[1] == 1 then
                        s = v[2]
                    elseif v[1] == - 1 then
                        s = "-" .. v[2]
                    else
                        s = string.format("%.3f",v[1]) .. v[2]
                    end
                end
            end
        end
        s = s or "0"
        return s
    end
    
    m["__concat"] = function (q,s)
        if is_a(s,"string") then
            return q:tostring() .. s
        else
            return q .. s:tostring()
        end
    end
    
    m["tomatrixleft"] = function (q)
        q = q:normalise()
        local a,b,c,d = q.w,q.x,q.y,q.z
        local ab,ac,ad,bb,bc,bd,cc,cd,dd = 2*a*b,2*a*c,2*a*d,2*b*b,2*b*c,2*b*d,2*c*c,2*c*d,2*d*d
        return matrix(
        1-cc-dd, bc-ad, ac+bd, 0,
        bc+ad, 1-bb-dd, cd-ab, 0,
        bd-ac, cd+ab, 1-bb-cc, 0,
        0,0,0,1
        )
    end
    
    m["tomatrixright"] = function (q)
        q = q:normalise()
        local a,b,c,d = q.w,-q.x,-q.y,-q.z
        local ab,ac,ad,bb,bc,bd,cc,cd,dd = 2*a*b,2*a*c,2*a*d,2*b*b,2*b*c,2*b*d,2*c*c,2*c*d,2*d*d
        return matrix(
        1-cc-dd, bc-ad, ac+bd, 0,
        bc+ad, 1-bb-dd, cd-ab, 0,
        bd-ac, cd+ab, 1-bb-cc, 0,
        0,0,0,1
        )
    end
    
    m["tomatrix"] = m["tomatrixright"]
    
    m["toangleaxis"] = function (q)
        q = q:normalise()
        local a = q.w
        q = vec3(q.x,q.y,q.z)
        if q == vec3(0,0,0) then
            return 0,vec3(0,0,1)
        end
        return 2*acos(a),q:normalise()
    end
    
    m["Gravity"] = function (q)
        local y = vec3(0,-1,0)^q
        return y:rotateTo(Gravity)*q
    end
    m.__extended = true
    
    for k,v in pairs(m) do
        rawset(mq,k,v)
    end
end


m = getmetatable(vec3())
if not m.__extended then
    m["is_finite"] = function(v)
        if v.x < huge
        and v.x > -huge
        and v.y < huge
        and v.y > -huge
        and v.z < huge
        and v.z > -huge
        then
            return true
        end
        return false
    end
    
    m["toquat"] = function (v)
        return quat(0,v.x,v.y,v.z)
    end
    
    m["applyquat"] = function (v,q)
        return q:__mul(v:toquat()):__mul(q:conjugate()):vector()
    end
    
    m["rotate"] = function(v,q,x,y,z)
        if is_a(q,"number") then
            q = quat.angleAxis(q,x,y,z)
        end
        return v:applyquat(q)
    end
    
    m["__pow"] = function (v,q)
        if is_a(q,quat) then
            return v:applyquat(q)
        end
        return false
    end
    
    m["__concat"] = function (u,s)
        if is_a(s,"string") then
            return u:__tostring() .. s
        else
            return u .. s:__tostring()
        end
    end
    
    m["rotateTo"] = function (u,v)
        return quat.fromToRotation(u,v)
    end
    
    m["normalise"] = function (v)
        v = v:normalize()
        if v:is_finite() then
            return v
        else
            return vec3(0,0,1)
        end
    end
    
    local mul3,add3,sub3 = m["__mul"],m["__add"],m["__sub"]
    m["__mul"] = function(m,v)
        if is_a(m,vec3)
        and is_a(v,"number")
        then
            return mul3(m,v)
        end
        if is_a(m,"number")
        and is_a(v,vec3)
        then
            return mul3(m,v)
        end
        if is_a(m,vec3)
        and is_a(v,vec3)
        then
            return vec3(m.x*v.x,m.y*v.y,m.z*v.z)
        end
        if is_a(m,matrix)
        and is_a(v,vec3)
        then
            local l = m[13]*v.x+m[14]*v.y+m[15]*v.z+m[16]
            return vec3(
            (m[1]*v.x + m[2]*v.y + m[3]*v.z + m[4])/l,
            (m[5]*v.x + m[6]*v.y + m[7]*v.z + m[8])/l,
            (m[9]*v.x + m[10]*v.y + m[11]*v.z + m[12])/l)
        end
        if is_a(m,vec3)
        and is_a(v,matrix)
        then
            local l = v[4]*m.x+v[8]*m.y+v[12]*m.z+v[16]
            return vec3(
            (v[1]*m.x + v[5]*m.y + v[9]*m.z + v[13])/l,
            (v[2]*m.x + v[6]*m.y + v[10]*m.z + v[14])/l,
            (v[3]*m.x + v[7]*m.y + v[11]*m.z + v[15])/l)
        end
    end
    
    m["__add"] = function(a,b)
        if is_a(a,"number") then
            a = vec3(a,a,a)
        end
        if is_a(b,"number") then
            b = vec3(b,b,b)
        end
        return add3(a,b)
    end
    
    m["__sub"] = function(a,b)
        if is_a(a,"number") then
            a = vec3(a,a,a)
        end
        if is_a(b,"number") then
            b = vec3(b,b,b)
        end
        return sub3(a,b)
    end
    
    m["exp"] = qTangent
    
    m["len1"] = function(c)
        return abs(c.x) + abs(c.y) + abs(c.z)
    end
    
    m["dist1"] = function(c,v)
        return abs(c.x - v.x) + abs(c.y - v.y) + abs(c.z - v.z)
    end
    
    m["leninf"] = function(c)
        return max(abs(c.x), abs(c.y), abs(c.z))
    end
    
    m["distinf"] = function(c,v)
        return max(abs(c.x - v.x), abs(c.y - v.y), abs(c.z - v.z))
    end
    
    m.__extended = true
end

m = getmetatable(matrix())
if not m.__extended then
    local mmul, mrotate = m["__mul"],m["rotate"]
    
    m["__mul"] = function (m,mm)
        if is_a(m,matrix)
        and is_a(mm,matrix)
        then
            return mmul(m,mm)
        end
        if is_a(m,matrix)
        and is_a(mm,quat)
        then
            return mmul(m,qQuat(mm):tomatrix())
        end
        if is_a(m,quat)
        and is_a(mm,matrix)
        then
            return mmul(qQuat(m):tomatrix(),mm)
        end
        if is_a(m,matrix)
        and is_a(mm,vec2)
        then
            return mmul(m,mm:tomatrix())
        end
        if is_a(m,vec2)
        and is_a(mm,matrix)
        then
            return mmul(m:tomatrix(),mm)
        end
        if is_a(m,matrix)
        and is_a(mm,vec3)
        then
            local l = m[13]*mm.x + m[14]*mm.y + m[15]*mm.z + m[16]
            return vec3(
            (m[1]*mm.x + m[2]*mm.y + m[3]*mm.z + m[4])/l,
            (m[5]*mm.x + m[6]*mm.y + m[7]*mm.z + m[8])/l,
            (m[9]*mm.x + m[10]*mm.y + m[11]*mm.z + m[12])/l)
        end
        if is_a(m,vec3)
        and is_a(mm,matrix)
        then
            local l = mm[4]*m.x + mm[8]*m.y + mm[12]*m.z + mm[16]
            return vec3(
            (mm[1]*m.x + mm[5]*m.y + mm[9]*m.z + mm[13])/l,
            (mm[2]*m.x + mm[6]*m.y + mm[10]*m.z + mm[14])/l,
            (mm[3]*m.x + mm[7]*m.y + mm[11]*m.z + mm[15])/l)
        end
    end
    
    m["rotate"] = function(m,a,x,y,z)
        if is_a(a,quat) then
            a,x = a:toangleaxis()
            x,y,z = x.x,x.y,x.z
        end
        return mrotate(m,a,x,y,z)
    end
    
    m.__extended = true
end


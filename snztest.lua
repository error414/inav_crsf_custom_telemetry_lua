local CRSF_FRAME_CUSTOM_TELEM   = 0x88
local SIMULATOR   = true

local function myprint (format, ...)
    local str = string.format("INAV: " .. format, ...)
    if SIMULATOR then
        print(str)
    else
        serialWrite(str .. "\r\n") 
    end
end


local function decNil(data, pos)
    return nil, pos
end

local function decU8(data, pos)
    return data[pos], pos+1
end

local function decS8(data, pos)
    local val,ptr = decU8(data,pos)
    return val < 0x80 and val or val - 0x100, ptr
end

local function decU16(data, pos)
    return bit32.lshift(data[pos],8) + data[pos+1], pos+2
end

local function decS16(data, pos)
    local val,ptr = decU16(data,pos)
    return val < 0x8000 and val or val - 0x10000, ptr
end

local function decU12U12(data, pos)
    local a = bit32.lshift(bit32.extract(data[pos],0,4),8) + data[pos+1]
    local b = bit32.lshift(bit32.extract(data[pos],4,4),8) + data[pos+2]
    return a,b,pos+3
end

local function decS12S12(data, pos)
    local a,b,ptr = decU12U12(data, pos)
    return a < 0x0800 and a or a - 0x1000, b < 0x0800 and b or b - 0x1000, ptr
end

local function decU24(data, pos)
    return bit32.lshift(data[pos],16) + bit32.lshift(data[pos+1],8) + data[pos+2], pos+3
end

local function decS24(data, pos)
    local val,ptr = decU24(data,pos)
    return val < 0x800000 and val or val - 0x1000000, ptr
end

local function decU32(data, pos)
    return bit32.lshift(data[pos],24) + bit32.lshift(data[pos+1],16) + bit32.lshift(data[pos+2],8) + data[pos+3], pos+4
end

local function decS32(data, pos)
    local val,ptr = decU32(data,pos)
    return val < 0x80000000 and val or val - 0x100000000, ptr
end

local function decCellV(data, pos)
    local val,ptr = decU8(data,pos)
    return val > 0 and val + 200 or 0, ptr
end

local function decCells(data, pos)
    local cnt,val,vol
    cnt,pos = decU8(data,pos)
    setTelemetryValue(0x1020, 0, 0, cnt, UNIT_RAW, 0, "Cel#")
    for i = 1, cnt
    do
        val,pos = decU8(data,pos)
        val = val > 0 and val + 200 or 0
        vol = bit32.lshift(cnt,24) + bit32.lshift(i-1, 16) + val
        setTelemetryValue(0x102F, 0, 0, vol, UNIT_CELLS, 2, "Cels")
    end
    return nil, pos
end

local function decAttitude(data, pos)
    local p,r,y
    p,pos = decS16(data,pos)
    r,pos = decS16(data,pos)
    y,pos = decS16(data,pos)
    setTelemetryValue(0x1101, 0, 0, p, UNIT_DEGREE, 1, "Ptch")
    setTelemetryValue(0x1102, 0, 0, r, UNIT_DEGREE, 1, "Roll")
    setTelemetryValue(0x1103, 0, 0, y, UNIT_DEGREE, 1, "Yaw")
    return nil, pos
end

local function decAccel(data, pos)
    local x,y,z
    x,pos = decS16(data,pos)
    y,pos = decS16(data,pos)
    z,pos = decS16(data,pos)
    setTelemetryValue(0x1111, 0, 0, x, UNIT_G, 2, "AccX")
    setTelemetryValue(0x1112, 0, 0, y, UNIT_G, 2, "AccY")
    setTelemetryValue(0x1113, 0, 0, z, UNIT_G, 2, "AccZ")
    return nil, pos
end

local function decLatLong(data, pos)
    local UNIT_GPS_LONGITUDE = 43
    local UNIT_GPS_LATITUDE = 44
    local lat,lon
    lat,pos = decS32(data,pos)
    lon,pos = decS32(data,pos)
    setTelemetryValue(0x1125, 0, 0, 0, UNIT_GPS, 0, "GPS")
    setTelemetryValue(0x1125, 0, 0, lat/10, UNIT_GPS_LATITUDE)
    setTelemetryValue(0x1125, 0, 0, lon/10, UNIT_GPS_LONGITUDE)
    return nil, pos
end



local sensorsById = {
    -- No data
    [0]  = { sid = 0x1000, name = "NONE", unit = UNIT_RAW, prec = 0, dec = decNil },
    -- Heartbeat (millisecond uptime % 60000)
    [1]  = { sid = 0x1001, name = "BEAT", unit = UNIT_RAW, prec = 0, dec = decU16 },
    -- Main battery voltage
    [2]  = { sid = 0x1011, name = "Vbat", unit = UNIT_VOLTS, prec = 2, dec = decU16 },
    -- Main battery current
    [3]  = { sid = 0x1012, name = "Curr", unit = UNIT_AMPS, prec = 2, dec = decU16 },
    -- Main battery used capacity
    [4]  = { sid = 0x1013, name = "Capa", unit = UNIT_MAH, prec = 0, dec = decU16 },
    -- Main battery charge / fuel level
    [5]  = { sid = 0x1014, name = "Bat%", unit = UNIT_PERCENT, prec = 0, dec = decU8 },
    -- Main battery cell count
    [6]  = { sid = 0x1020, name = "Cel#", unit = UNIT_RAW, prec = 0, dec = decU8 },
    -- Main battery cell voltage (minimum/average)
    [7]  = { sid = 0x1021, name = "Vcel", unit = UNIT_VOLTS, prec = 2, dec = decCellV },
    -- Main battery cell voltages
    [8]  = { sid = 0x102F, name = "Cels", unit = UNIT_VOLTS, prec = 2, dec = decCells },
    -- Variometer (combined baro+GPS)
    [9]  = { sid = 0x10B3, name = "Var", unit = UNIT_METERS_PER_SECOND, prec = 2, dec = decS16 },
    -- Heading (combined gyro+mag+GPS)
    [10] = { sid = 0x10B1, name = "Hdg", unit = UNIT_DEGREE, prec = 1, dec = decS16 },
    -- Altitude (combined baro+GPS)
    [11] = { sid = 0x10B2, name = "Alt", unit = UNIT_METERS, prec = 2, dec = decS24 },
    -- Attitude (hires combined)
    [12] = { sid = 0x1100, name = "Attd", unit = UNIT_DEGREE, prec = 1, dec = decAttitude },
    -- Attitude pitch
    [13] = { sid = 0x1101, name = "Ptch", unit = UNIT_DEGREE, prec = 0, dec = decS16 },
    -- Attitude roll
    [14] = { sid = 0x1102, name = "Roll", unit = UNIT_DEGREE, prec = 0, dec = decS16 },
    -- Attitude yaw
    [15] = { sid = 0x1103, name = "Yaw", unit = UNIT_DEGREE, prec = 0, dec = decS16 },
    -- Acceleration X
    [16] = { sid = 0x1111, name = "AccX", unit = UNIT_G, prec = 1, dec = decS16 },
    -- Acceleration Y
    [17] = { sid = 0x1112, name = "AccY", unit = UNIT_G, prec = 1, dec = decS16 },
    -- Acceleration Z
    [18] = { sid = 0x1113, name = "AccZ", unit = UNIT_G, prec = 1, dec = decS16 },
    -- GPS Satellite count
    [19] = { sid = 0x1121, name = "Sats", unit = UNIT_RAW, prec = 0, dec = decU8 },
    -- GPS HDOP
    [20] = { sid = 0x1123, name = "HDOP", unit = UNIT_RAW, prec = 0, dec = decU8 },
    -- GPS Coordinates
    [21] = { sid = 0x1125, name = "GPS", unit = UNIT_RAW, prec = 0, dec = decLatLong },
    -- GPS altitude
    [22] = { sid = 0x1126, name = "GAlt", unit = UNIT_METERS, prec = 1, dec = decS16 },
    -- GPS heading
    [23] = { sid = 0x1127, name = "GHdg", unit = UNIT_DEGREE, prec = 1, dec = decS16 },
    -- GPS ground speed
    [24] = { sid = 0x1128, name = "GSpd", unit = UNIT_METERS_PER_SECOND, prec = 2, dec = decU16 },
    -- GPS home distance
    [25] = { sid = 0x1129, name = "GDis", unit = UNIT_METERS, prec = 1, dec = decU16 },
    -- GPS home direction
    [26] = { sid = 0x112A, name = "GDir", unit = UNIT_METERS, prec = 1, dec = decU16 },
    -- CPU load
    [27] = { sid = 0x1141, name = "CPU%", unit = UNIT_PERCENT, prec = 0, dec = decU8 },
    -- Flight mode flags
    [28] = { sid = 0x1201, name = "Mode", unit = UNIT_RAW, prec = 0, dec = decU16 },
    -- Arming flags
    [29] = { sid = 0x1202, name = "ARM", unit = UNIT_RAW, prec = 0, dec = decU8 },
}

local function getSensorBySID(sid)
    for _, sensor in pairs(sensorsById) do
        if sensor.sid == sid then
            return sensor
        end
    end
    return nil -- SID nenalezen
end


local telemetryFrameId = 0
local telemetryFrameSkip = 0
local telemetryFrameCount = 0

local function crossfirePop()
    local command
    local data
    if SIMULATOR then
        command = 0x88
        data = {0xEA, 0xC8, 0x91, 0x11, 0x26, 0x00, 0x00, 0x11, 0x27, 0x00, 0x00, 0x11, 0x28, 0x00, 0x00, 0x11, 0x29, 0x00, 0x00, 0x11, 0x2A, 0x00, 0x00, 0x11, 0x00, 0xFF, 0xF9, 0xFF, 0xFE, 0x0D, 0xCA}
    else
        command, data = crossfireTelemetryPop()
    end
    
    if command and data then
        --myprint("command: %s", command)
        if command == CRSF_FRAME_CUSTOM_TELEM then
            local fid, sid, val
            local ptr = 3
            fid,ptr = decU8(data, ptr)
            local delta = bit32.band(fid - telemetryFrameId, 0xFF)
            -- myprint("delta: %s", delta)
            if delta > 1 then
                telemetryFrameSkip = telemetryFrameSkip + 1
            end
            telemetryFrameId = fid
            telemetryFrameCount = telemetryFrameCount + 1
            while ptr < #data do
                sid,ptr = decU16(data, ptr)
                local sensor = getSensorBySID(sid)
                myprint("sid: 0x%04X", sid)
                if sensor then
                    val,ptr = sensor.dec(data, ptr)
                    if val then
                        myprint("----")
                        myprint("sid: 0x%04X", sid)
                        myprint("name: %s", sensor.name)
                        myprint("val: %s", val)
                        myprint("unit: %s", sensor.unit)
                        myprint("----")
                        setTelemetryValue(sid, 0, 0, val, sensor.unit, sensor.prec, sensor.name)
                    end
                else
                    break
                end
            end
            setTelemetryValue(0xEE01, 0, 0, telemetryFrameCount, UNIT_RAW, 0, "*Cnt")
            setTelemetryValue(0xEE02, 0, 0, telemetryFrameSkip, UNIT_RAW, 0, "*Skp")
            setTelemetryValue(0xEE03, 0, 0, sid, UNIT_RAW, 0, "*Frm")
        end
        return true
    end
    return false
end

local function run(event)
    if SIMULATOR then
        crossfirePop()
    else
        while crossfirePop() do end
    end
    return 0
end


return { run = run }
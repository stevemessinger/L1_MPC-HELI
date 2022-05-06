//This file was generated by luaexport.py script on Tue May  3 10:00:20 2022 for OpenTX version 2.3.15



    struct LuaSingleField {
        uint16_t id;
        const char * name;
        const char * desc;
    };

    struct LuaMultipleField {
        uint16_t id;
        const char * name;
        const char * desc;
        uint8_t count;
    };

    
    // The list of Lua fields
    // this aray is alphabetically sorted by the second field (name)
    const LuaSingleField luaSingleFields[] = {
        {MIXSRC_Ail, "ail", "Aileron"},
    {MIXSRC_TX_TIME, "clock", "RTC clock [minutes from midnight]"},
    {MIXSRC_CYC1, "cyc1", "Cyclic 1"},
    {MIXSRC_CYC2, "cyc2", "Cyclic 2"},
    {MIXSRC_CYC3, "cyc3", "Cyclic 3"},
    {MIXSRC_Ele, "ele", "Elevator"},
    {MIXSRC_MAX, "max", "MAX"},
    {MIXSRC_Rud, "rud", "Rudder"},
    {MIXSRC_POT1, "s1", "Potentiometer 1"},
    {MIXSRC_POT2, "s2", "Potentiometer 2"},
    {MIXSRC_SA, "sa", "Switch A"},
    {MIXSRC_SB, "sb", "Switch B"},
    {MIXSRC_SC, "sc", "Switch C"},
    {MIXSRC_SD, "sd", "Switch D"},
    {MIXSRC_SF, "sf", "Switch F"},
    {MIXSRC_SH, "sh", "Switch H"},
    {MIXSRC_SI, "si", "Switch I"},
    {MIXSRC_SJ, "sj", "Switch J"},
    {MIXSRC_Thr, "thr", "Throttle"},
    {MIXSRC_TIMER1, "timer1", "Timer 1 value [seconds]"},
    {MIXSRC_TIMER2, "timer2", "Timer 2 value [seconds]"},
    {MIXSRC_TIMER3, "timer3", "Timer 3 value [seconds]"},
    {MIXSRC_TrimAil, "trim-ail", "Aileron trim"},
    {MIXSRC_TrimEle, "trim-ele", "Elevator trim"},
    {MIXSRC_TrimRud, "trim-rud", "Rudder trim"},
    {MIXSRC_TrimThr, "trim-thr", "Throttle trim"},
    {MIXSRC_TX_VOLTAGE, "tx-voltage", "Transmitter battery voltage [volts]"}
};


    // The list of Lua fields that have a range of values
    const LuaMultipleField luaMultipleFields[] = {
        {MIXSRC_FIRST_INPUT, "input", "Input [I%d]", 32},
    {MIXSRC_FIRST_LUA, "lua", "Lua mix output %d", 42},
    {MIXSRC_SW1, "ls", "Logical switch L%d", 64},
    {MIXSRC_FIRST_TRAINER, "trn", "Trainer input %d", 16},
    {MIXSRC_CH1, "ch", "Channel CH%d", 32},
    {MIXSRC_GVAR1, "gvar", "Global variable %d", 9},
    {MIXSRC_FIRST_TELEM, "telem", "Telemetry sensor %d", 40}
};

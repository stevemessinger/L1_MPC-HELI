/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

// ES translations authors: Jose Moreno <josemoreno259@gmail.com>, Daniel GeA <daniel.gea.1000@gmail.com>

/*
 * !!!!! DO NOT EDIT es.h - EDIT es.h.txt INSTEAD !!!!!!!
 *
 * In order to make translations easier es.h.txt is parsed and national
 * characters are replaced by bitmap codes. The result is es.h.
 *
 * See translate.py in the util folder for the list of character codes
 *
 * !!!!! DO NOT EDIT es.h - EDIT es.h.txt INSTEAD !!!!!!!
 */

/* Formatting octal codes available in TR_ strings:
 *  \037\x           -sets LCD x-coord (x value in octal)
 *  \036             -newline
 *  \035             -horizontal tab (ARM only)
 *  \001 to \034     -extended spacing (value * FW/2)
 *  \0               -ends current string
 */



// NON ZERO TERMINATED STRINGS
#define LEN_OFFON              "\003"
#define TR_OFFON               "OFF""ON\0"

#define LEN_MMMINV             "\003"
#define TR_MMMINV              "---""INV"

#define LEN_VBEEPMODE          "\010"
#define TR_VBEEPMODE           "Silencio""Alarma\0 ""No tecla""Todo\0   "

#define LEN_VBLMODE            "\006"
#define TR_VBLMODE             "OFF\0  ""Teclas""Sticks""Ambos\0""ON\0   "

#define LEN_TRNMODE            "\003"
#define TR_TRNMODE             "OFF""+=\0"":="

#define LEN_TRNCHN             "\003"
#define TR_TRNCHN              "CH1CH2CH3CH4"

#define LEN_AUX_SERIAL_MODES   "\015"
#if defined(CLI) || defined(DEBUG)
#define TR_AUX_SERIAL_MODES    "Debug\0       ""Telem Mirror\0""Telemetr\207a\0  ""Entrenador SBUS""LUA\0         "
#else
#define TR_AUX_SERIAL_MODES    "OFF\0         ""Telem Mirror\0""Telemetr\207a\0  ""Entrenador SBUS""LUA\0         "
#endif

#define LEN_SWTYPES            "\007"
#define TR_SWTYPES             "Nada\0  ""Palanca""2POS\0  ""3POS\0  "

#define LEN_POTTYPES           TR("\013", "\017")
#define TR_POTTYPES            TR("Nada\0      ""Pot con fij""Multipos\0  ""Pot\0       ", "Nada\0          ""Pot con fijador""Switch multipos""Pot\0           ")

#define LEN_SLIDERTYPES        "\006"
#define TR_SLIDERTYPES         "Nada\0 ""Slider"

#define LEN_VLCD               "\006"
#define TR_VLCD                "NormalOptrex"

#define LEN_VPERSISTENT        "\014"
#define TR_VPERSISTENT         "OFF\0        ""Vuelo\0      ""Reset manual"

#define LEN_COUNTRYCODES       TR("\002", "\007")
#define TR_COUNTRYCODES        TR("US""JP""EU", "Am\205rica""Jap\211n\0 ""Europa\0")

#if defined(RADIO_FAMILY_TBS)
#define LEN_USBMODES                   TR("\006", "\010")
#define TR_USBMODES                    TR("Ask\0  ""Joyst\0""Agent\0""SDCard""Serial", "Ask\0    ""Joystick""Agent\0 ""Storage\0""Serial\0 ")
#else
#define LEN_USBMODES           "\010"
#if defined(DEBUF)
#define TR_USBMODES            "Pregunta""Joystick""SDCard\0 ""Serie\0  "
#else
#define TR_USBMODES            "Pregunta""Joystick""SDCard\0 ""Telem\0  "
#endif
#endif

#define LEN_JACKMODES          "\010"
#define TR_JACKMODES           "Pregunta""Audio\0  ""Entrena."

#define LEN_TELEMETRY_PROTOCOLS "\017"
#define TR_TELEMETRY_PROTOCOLS "FrSky S.PORT\0  ""FrSky D\0       ""FrSky D (cable)""TBS Crossfire\0 ""Spektrum\0      ""AFHDS2A IBUS\0  ""Multi Telemetr."

#define TR_MULTI_CUSTOM        "Custom"

#define LEN_VTRIMINC           TR("\006", "\013")
#define TR_VTRIMINC            TR("Expo\0 ""ExFino""Fino\0 ""Medio\0""Grueso", "Exponencial""Extra fino\0""Fino\0      ""Medio\0     ""Grueso\0    ")

#define LEN_VDISPLAYTRIMS      "\007"
#define TR_VDISPLAYTRIMS       "No\0    ""Cambiar""Si\0    "

#define LEN_VBEEPCOUNTDOWN     "\010"
#define TR_VBEEPCOUNTDOWN      "Silencio""Beeps\0  ""Voz\0    ""Haptic\0  "

#define LEN_VVARIOCENTER       "\010"
#define TR_VVARIOCENTER        "Tono\0   ""Silencio"

#define LEN_CURVE_TYPES        "\006"
#define TR_CURVE_TYPES         "Normal""Custom"

#define LEN_RETA123            "\001"

#if defined(PCBHORUS)
  #define TR_RETA123           "RETA13245LR"
#elif defined(PCBX9E)
  #define TR_RETA123           "RETA1234LRLR"
#elif defined(PCBTARANIS) || defined(REVX)
  #define TR_RETA123           "RETA123LR"
#elif defined(PCBSKY9X)
  #define TR_RETA123           "RETA123a"
#else
  #define TR_RETA123           "RETA123"
#endif

#if defined(PCBSKY9X) && defined(REVX)
  #define LEN_VOUTPUT_TYPE     "\011"
  #define TR_VOUTPUT_TYPE      "OpenDrain""PushPull\0"
#endif

#define LEN_VCURVEFUNC         "\003"
#define TR_VCURVEFUNC          "---""x>0""x<0""|x|""f>0""f<0""|f|"

#define LEN_VMLTPX             "\010"
#define TR_VMLTPX              "A\201adir\0 ""Multipl.""Cambiar\0"

#define LEN_VMLTPX2            "\002"
#define TR_VMLTPX2             "+=""*="":="

#define LEN_VMIXTRIMS          "\003"

#if defined(PCBHORUS)
  #define TR_VMIXTRIMS         "OFF""ON\0""Rud""Ele""Thr""Ail""T5\0""T6\0"
#else
  #define TR_VMIXTRIMS         "OFF""ON\0""Rud""Ele""Thr""Ail"
#endif

#if LCD_W >= 212
  #define TR_CSWTIMER          "Timer"
  #define TR_CSWSTICKY         "Pega\0"
  #define TR_CSWRANGE          "Rango"
  #define TR_CSWSTAY           "Borde"
#else
  #define TR_CSWTIMER          "Tim\0 "
  #define TR_CSWSTICKY         "Pega\0"
    #define TR_CSWRANGE        "Rngo\0"
    #define TR_CSWSTAY         "Bord\0"
#endif

  #define TR_CSWEQUAL      "a=x\0 "

#define LEN_VCSWFUNC           "\005"
#define TR_VCSWFUNC            "---\0 " TR_CSWEQUAL "a\173x\0 ""a>x\0 ""a<x\0 " TR_CSWRANGE "|a|>x""|a|<x""AND\0 ""OR\0  ""XOR\0 " TR_CSWSTAY "a=b\0 ""a>b\0 ""a<b\0 ""\306}x\0 ""|\306|}x" TR_CSWTIMER TR_CSWSTICKY

#define LEN_VFSWFUNC           "\012"

#if defined(VARIO)
  #define TR_VVARIO            "Vario\0    "
#else
  #define TR_VVARIO            "[Vario]\0  "
#endif

#if defined(AUDIO)
  #define TR_SOUND             "Oir sonido"
#else
  #define TR_SOUND             "Beep\0     "
#endif

#if defined(HAPTIC)
  #define TR_HAPTIC            "Haptic\0   "
#else
  #define TR_HAPTIC            "[Haptic]\0 "
#endif

#if defined(VOICE)
  #if defined(PCBSKY9X)
    #define TR_PLAY_TRACK      "Oir\0      "
  #else
    #define TR_PLAY_TRACK      "Oir pista\0"
  #endif
  #define TR_PLAY_BOTH         "Oir Ambos\0"
  #define TR_PLAY_VALUE        "Oir valor\0"
#else
  #define TR_PLAY_TRACK        "[OirPista]"
  #define TR_PLAY_BOTH         "[OirAmbos]"
  #define TR_PLAY_VALUE        "[OirValor]"
#endif

#define TR_SF_BG_MUSIC        "BgM\213sica\0 ""BgM\213sica||"

#if defined(SDCARD)
  #define TR_SDCLOGS           "SD Logs\0  "
#else
  #define TR_SDCLOGS           "[SD Logs]\0"
#endif

#if defined(GVARS)
  #define TR_ADJUST_GVAR       "Ajuste\0   "
#else
  #define TR_ADJUST_GVAR       "[AjusteGV]"
#endif

#if defined(LUA)
  #define TR_SF_PLAY_SCRIPT   "Lua Script"
#else
  #define TR_SF_PLAY_SCRIPT   "[Lua]\0    "
#endif

#if defined(DEBUG)
  #define TR_SF_TEST          "Test\0"
#else
  #define TR_SF_TEST
#endif

#if defined(OVERRIDE_CHANNEL_FUNCTION)
  #define TR_SF_SAFETY        "Seguro\0   "
#else
  #define TR_SF_SAFETY        "---\0      "
#endif

#define TR_SF_SCREENSHOT      "Captura\0  "
#define TR_SF_RACING_MODE     "RacingMode"
#define TR_SF_RESERVE         "[reserv.]\0"

#define TR_VFSWFUNC            TR_SF_SAFETY "Entrenador""Inst. Trim""Reset\0    ""Ajuste \0  " TR_ADJUST_GVAR "Volumen\0  " "Failsafe\0 " "CheckRango" "Enl.m\211dulo" TR_SOUND TR_PLAY_TRACK TR_PLAY_VALUE TR_SF_RESERVE TR_SF_PLAY_SCRIPT TR_SF_RESERVE TR_SF_BG_MUSIC TR_VVARIO TR_HAPTIC TR_SDCLOGS "Luz Fondo\0" TR_SF_SCREENSHOT TR_SF_RACING_MODE TR_SF_TEST

#define LEN_VFSWRESET          TR("\004", "\012")
#define TR_FSW_RESET_TELEM     TR("Telm", "Telemetr\207a")

#if LCD_W >= 212
  #define TR_FSW_RESET_TIMERS  "Timer 1\0  ""Timer 2\0  ""Timer 3\0  "
#else
  #define TR_FSW_RESET_TIMERS  "Tmr1""Tmr2""Tmr3"
#endif

#define TR_VFSWRESET           TR(TR_FSW_RESET_TIMERS "Todo" TR_FSW_RESET_TELEM, TR_FSW_RESET_TIMERS "Vuelo\0    " TR_FSW_RESET_TELEM)

#define LEN_FUNCSOUNDS         TR("\004", "\006")
#define TR_FUNCSOUNDS          TR("Bp1\0""Bp2\0""Bp3\0""Avs1""Avs2""Chee""Rata""Tick""Sirn""Ring""SciF""Robt""Chrp""Tada""Crck""Alrm", "Beep1 ""Beep2 ""Beep3 ""Aviso1""Aviso2""Cheep ""Ratata""Tick  ""Sirena""Ring  ""SciFi ""Robot ""Chirp ""Tada  ""Crickt""AlmClk")

#define LEN_VTELEMCHNS         "\004"

#define LENGTH_UNIT_IMP        "ft\0"
#define SPEED_UNIT_IMP         "mph"
#define LENGTH_UNIT_METR       "m\0 "
#define SPEED_UNIT_METR        "kmh"

#define LEN_VUNITSSYSTEM     "\010"
#define TR_VUNITSSYSTEM      "M\205trico\0""Imperial"
#define LEN_VTELEMUNIT       "\003"
#define TR_VTELEMUNIT        "-\0 ""V\0 ""A\0 ""mA\0""kts""m/s""f/s""kmh""mph""m\0 ""ft\0""@C\0""@F\0""%\0 ""mAh""W\0 ""mW\0""dB\0""rpm""g\0 ""@\0 ""rad""ml\0""fOz""mlm""Hz\0""mS\0""uS\0""km\0""dbm"

#define STR_V                  (STR_VTELEMUNIT+1)
#define STR_A                  (STR_VTELEMUNIT+4)

#define LEN_VTELEMSCREENTYPE   "\007"
#define TR_VTELEMSCREENTYPE    "Nada\0  ""N\213meros""Barras\0""Script\0"

#define LEN_GPSFORMAT          "\004"
#define TR_GPSFORMAT           "HMS NMEA"

#define LEN2_VTEMPLATES        12
#define LEN_VTEMPLATES         "\014"
#define TR_TEMPLATE_CLEAR_MIXES        "Elim Mezcla\0"
#define TR_TEMPLATE_SIMPLE_4CH         "Simple 4-CH "
#define TR_TEMPLATE_STICKY_TCUT        "Anular Motor"
#define TR_TEMPLATE_VTAIL              "Cola en V   "
#define TR_TEMPLATE_DELTA              "Elevon\\Delta"
#define TR_TEMPLATE_ECCPM              "eCCPM       "
#define TR_TEMPLATE_HELI               "Heli Setup  "
#define TR_TEMPLATE_SERVO_TEST         "Servo Test  "

#define LEN_VSWASHTYPE         "\004"
#define TR_VSWASHTYPE          "---\0""120\0""120X""140\0""90\0"

#if defined(PCBHORUS)
  #define LEN_VKEYS                    "\005"
  #define TR_VKEYS                     "PGUP\0""PGDN\0""ENTER""MDL\0 ""RTN\0 ""TELE\0""SYS\0 "
#elif defined(PCBXLITE)
  #define LEN_VKEYS                    "\005"
  #define TR_VKEYS                     "Shift""Exit\0""Enter""Down\0""Up\0  ""Right""Left\0"
#elif defined(RADIO_FAMILY_JUMPER_T12)
  #define LEN_VKEYS                    "\005"
  #define TR_VKEYS                     "Exit\0""Enter""Down\0""Up\0  ""Right""Left\0"
#elif defined(RADIO_TX12)
  #define LEN_VKEYS                    "\005"
  #define TR_VKEYS                     "Exit\0""Enter""Up\0  ""Down\0""SYS\0 ""MDL\0 ""TELE\0"
#elif defined(RADIO_T8)
  #define LEN_VKEYS                    "\005"
  #define TR_VKEYS                     "RTN\0 ""ENTER""PGUP\0""PGDN\0""SYS\0 ""MDL\0 ""UP\0  ""DOWN\0"
#elif defined(PCBTARANIS)
  #define LEN_VKEYS                    "\005"
  #define TR_VKEYS                     "Menu\0""Exit\0""Enter""Page\0""Plus\0""Minus"
#else
  #define LEN_VKEYS                    "\005"
  #define TR_VKEYS                     "Menu\0""Exit\0""Down\0""Up\0  ""Right""Left\0"
#endif

#define LEN_VSWITCHES          "\003"
#define LEN_VSRCRAW            "\004"

#define TR_STICKS_VSRCRAW      "\307Rud""\307Ele""\307Thr""\307Ail"

#if defined(PCBHORUS)
  #define TR_TRIMS_VSRCRAW     "\313Rud""\313Ele""\313Thr""\313Ail""\313T5\0""\313T6\0"
#else
  #define TR_TRIMS_VSRCRAW     TR("TrmR" "TrmE" "TrmT" "TrmA", "\313Rud""\313Ele""\313Thr""\313Ail")
#endif

#if defined(PCBHORUS)
  #define TR_TRIMS_SWITCHES    "\313Rl""\313Rr""\313Ed""\313Eu""\313Td""\313Tu""\313Al""\313Ar""\3135d""\3135u""\3136d""\3136u"
#else
  #define TR_TRIMS_SWITCHES    TR("tRl""tRr""tEd""tEu""tTd""tTu""tAl""tAr", "\313Rl""\313Rr""\313Ed""\313Eu""\313Td""\313Tu""\313Al""\313Ar")
#endif

#if defined(PCBSKY9X)
  #define TR_ROTARY_ENCODERS   "REa\0"
  #define TR_ROTENC_SWITCHES   "REa"
#else
  #define TR_ROTARY_ENCODERS
  #define TR_ROTENC_SWITCHES
#endif

#define TR_ON_ONE_SWITCHES     "ON\0""One"

#if defined(GYRO)
  #define TR_GYR_VSRCRAW       "GyrX""GyrY"
#else
  #define TR_GYR_VSRCRAW
#endif

#if defined(HELI)
  #define TR_CYC_VSRCRAW       "CYC1""CYC2""CYC3"
#else
  #define TR_CYC_VSRCRAW       "[C1]""[C2]""[C3]"
#endif

#define TR_RESERVE_VSRCRAW   "[--]"
#define TR_EXTRA_VSRCRAW     "Bat\0""Time""GPS\0" TR_RESERVE_VSRCRAW TR_RESERVE_VSRCRAW TR_RESERVE_VSRCRAW TR_RESERVE_VSRCRAW "Tmr1""Tmr2""Tmr3"

#define LEN_VTMRMODES          "\003"
#define TR_VTMRMODES           "OFF""ABS""THs""TH%""THt"

#define LEN_VTRAINERMODES              "\022"
#define TR_VTRAINER_MASTER_JACK        "Master/Jack\0      "
#define TR_VTRAINER_SLAVE_JACK         "Esclav/Jack\0      "
#define TR_VTRAINER_MASTER_SBUS_MODULE "Master/M\211dulo SBUS"
#define TR_VTRAINER_MASTER_CPPM_MODULE "Master/M\211dulo CPPM"
#define TR_VTRAINER_MASTER_BATTERY     "Master/Serie\0     "
#define TR_VTRAINER_BLUETOOTH          TR("Master/BT\0        ""Esclavo/BT\0       ", "Master/Bluetooth\0 ""Esclavo/Bluetooth\0")
#define TR_VTRAINER_MULTI              "Master/Multi\0     "
#define TR_VTRAINER_SPORT_SBUS         "Master/SBUS-Sport\0"

#define LEN_VFAILSAFE          "\011"
#define TR_VFAILSAFE           "No\0      ""Hold\0    ""Custom\0  ""No pulsos""Receptor\0"


#define LEN_VSENSORTYPES        "\012"
#define TR_VSENSORTYPES        "Custom\0   ""Calculado"

#define LEN_VFORMULAS          "\011"
#define TR_VFORMULAS           "Suma\0    ""Media\0   ""Min\0     ""M\203x\0     ""Multipl. ""Total\0   ""Cell\0    ""Consumo\0 ""Distancia"

#define LEN_VPREC              "\004"
#define TR_VPREC               "0.--""0.0 ""0.00"

#define LEN_VCELLINDEX         "\007"
#define TR_VCELLINDEX          "Menor\0 ""1\0     ""2\0     ""3\0     ""4\0     ""5\0     ""6\0     ""Mayor\0 ""Delta\0"

#define LEN_GYROS                      "\004"
#define TR_GYROS                       "GyrX""GyrY"

#define LEN_TEXT_SIZE          "\013"
#define TR_TEXT_SIZE           "Normal\0    ""Muy peque\201o""Peque\201o\0   ""Medio\0     ""Doble\0     "

// ZERO TERMINATED STRINGS
#if defined(COLORLCD)
  #define INDENT                       "   "
  #define LEN_INDENT                   3
  #define INDENT_WIDTH                 12
  #define BREAKSPACE                   "\036"
#else
  #define INDENT                       "\001"
  #define LEN_INDENT                   1
  #define INDENT_WIDTH                 (FW/2)
  #define BREAKSPACE                   " "
#endif

#if defined(PCBTARANIS) || defined(PCBHORUS)
  #define TR_ENTER                     "[ENTER]"
#else
  #define TR_ENTER                     "[MENU]"
#endif

#if defined(PCBHORUS)
  #define TR_EXIT                      "[RTN]"
  #define TR_OK                        TR_ENTER
#else
  #define TR_EXIT                      "[EXIT]"
  #define TR_OK                        TR("\010" "\010" "\010" "[OK]", "\010" "\010" "\010" "\010" "\010" "[OK]")
#endif

#if defined(PCBTARANIS)
  #define TR_POPUPS_ENTER_EXIT         TR(TR_EXIT "\010" TR_ENTER, TR_EXIT "\010" "\010" "\010" "\010" TR_ENTER)

#else
  #define TR_POPUPS_ENTER_EXIT         TR_ENTER "\010" TR_EXIT
#endif

#define TR_MENUWHENDONE        CENTER "\007" TR_ENTER " AL ACABAR "
#define TR_FREE                "libre"
#define TR_DELETEMODEL         "BORRAR MODELO"
#define TR_COPYINGMODEL        "Copiando modelo.."
#define TR_MOVINGMODEL         "Moviendo modelo..."
#define TR_LOADINGMODEL        "Cargando modelo..."
#define TR_NAME                "Nombre"
#define TR_MODELNAME           TR("Nom. modelo", "Nombre modelo")
#define TR_PHASENAME           "Nombre fase "
#define TR_MIXNAME             TR("Nom. mezcla", "Nombre mezcla")
#define TR_INPUTNAME           TR("Entrada", "Nom. entrada")
#define TR_EXPONAME            TR("Nom.", "Nombre expo")
#define TR_BITMAP              "Imagen modelo"
#define TR_TIMER               TR("Timer", "Timer ")
#define TR_ELIMITS             TR("E.L\207mite", "Ampliar l\207mites")
#define TR_ETRIMS              TR("E.Trims", "Ampliar trims")
#define TR_TRIMINC             "Paso trim"
#define TR_DISPLAY_TRIMS       "Ver trims"
#define TR_TTRACE              TR("Fuente-A", INDENT "Fuente acelerador")
#define TR_TTRIM               TR("Trim-A", INDENT "Trim acelerador")
#define TR_TTRIM_SW            TR("T-Trim-Sw", INDENT "Trim switch")
#define TR_BEEPCTR             TR("Beep ctr", "Beep centro")
#define TR_USE_GLOBAL_FUNCS    TR("Funcs. glob.", "Usar func. globales")
#if defined(PCBSKY9X) && defined(REVX)
  #define TR_OUTPUT_TYPE       INDENT "Output"
#endif
#define TR_PROTOCOL            TR("Proto", "Protocolo")
#define TR_PPMFRAME            INDENT "Trama PPM"
#define TR_REFRESHRATE         TR(INDENT "Refresco", INDENT "Velocidad refresco")
#define STR_WARN_BATTVOLTAGE   TR(INDENT "Salida es VBAT: ", INDENT "Aviso: se\201al salida es VBAT: ")
#define TR_WARN_5VOLTS                 "Aviso: nivel de salida 5 voltios"
#define TR_MS                  "ms"
#define TR_FREQUENCY           INDENT "Frequencia"
#define TR_SWITCH              TR("Interr.", "Interruptor")
#define TR_TRIMS               "Trims"
#define TR_FADEIN              "Inicio"
#define TR_FADEOUT             "Final"
#define TR_DEFAULT             "(defecto)"
#define TR_CHECKTRIMS          CENTER "\006Check\012Trims"
#define OFS_CHECKTRIMS         CENTER_OFS+(9*FW)
#define TR_SWASHTYPE           "Tipo c\207clico"
#define TR_COLLECTIVE          TR("Colectivo", "Fuente colectivo")
#define TR_AILERON             TR("Col. lateral", "Fuente col. lateral")
#define TR_ELEVATOR            TR("Col. long. ", "Fuente col. longitudinal")
#define TR_SWASHRING           "Ciclico"
#define TR_ELEDIRECTION        TR("ELE Direcci\211n", "Largo cyc. direcci\211n")
#define TR_AILDIRECTION        TR("AIL Direcci\211n", "Lateral cyc. direcci\211n")
#define TR_COLDIRECTION        TR("PIT Direcci\211n", "Coll. pitch direcci\211n")
#define TR_MODE                "Modo"
#define TR_SUBTYPE             INDENT "Subtipo"
#define TR_NOFREEEXPO          "No expo libre!"
#define TR_NOFREEMIXER         "No mezcla lib!"
#define TR_SOURCE              "Fuente"
#define TR_WEIGHT              "Cantidad"
#define TR_EXPO                TR("Expo", "Exponencial")
#define TR_SIDE                "Zona"
#define TR_DIFFERENTIAL        "Diferenc"
#define TR_OFFSET              "Offset"
#define TR_TRIM                "Trim"
#define TR_DREX                "DRex"
#define DREX_CHBOX_OFFSET      30
#define TR_CURVE               "Curva"
#define TR_FLMODE              TR("Modo", "Modos")
#define TR_MIXWARNING          "Aviso"
#define TR_OFF                 "OFF"
#define TR_ANTENNA             "Antena"
#define TR_NO_INFORMATION      TR("Sin info", "Sin informaci\211n")
#define TR_MULTPX              "Multipx"
#define TR_DELAYDOWN           "Atraso bajar"
#define TR_DELAYUP             "Atraso subir"
#define TR_SLOWDOWN            "Bajar lento "
#define TR_SLOWUP              "Subir lento"
#define TR_MIXES               "MIXES"
#define TR_CV                  "CV"
#define TR_GV                  TR("G", "GV")
#define TR_ACHANNEL            "A\004canal"
#define TR_RANGE               INDENT"Alcance"
#define TR_CENTER              INDENT "Centro"
#define TR_BAR                 "Bar"
#define TR_ALARM               INDENT"Alarma"
#define TR_USRDATA             "UsrData"
#define TR_BLADES              INDENT"Palas"
#define TR_SCREEN              "Pant. "
#define TR_SOUND_LABEL         "Sonido"
#define TR_LENGTH              INDENT"Longitud"
#define TR_BEEP_LENGTH         INDENT "Duraci\211n Beep"
#define TR_SPKRPITCH           INDENT"Tono"
#define TR_HAPTIC_LABEL        "Haptic"
#define TR_HAPTICSTRENGTH      INDENT"Intensidad"
#define TR_GYRO_LABEL                  "Gyro"
#define TR_GYRO_OFFSET                 "Offset"
#define TR_GYRO_MAX                    "M\203x"
#define TR_CONTRAST            "Contraste"
#define TR_ALARMS_LABEL        "Alarmas"
#define TR_BATTERY_RANGE       TR("Rango bater\207a", "Rango medidor bater\207a")
#define TR_BATTERYWARNING      INDENT"Bater\207a baja"
#define TR_INACTIVITYALARM     INDENT"Inactividad"
#define TR_MEMORYWARNING       INDENT"Memoria baja"
#define TR_ALARMWARNING        INDENT"Sin sonido"
#define TR_RSSISHUTDOWNALARM   TR(INDENT "Apagado Rssi", INDENT "Check Rssi al apagar")
#define TR_MODEL_STILL_POWERED "Modelo aun encendido"
#define TR_MODEL_SHUTDOWN              "Apagar ?"
#define TR_PRESS_ENTER_TO_CONFIRM      "Enter para confirmar"
#define TR_THROTTLE_LABEL      "Potencia"
#define TR_THROTTLEREVERSE     TR("Invert acel", INDENT "Invertir acel.")
#define TR_MINUTEBEEP          TR("Minuto", "Cada minuto")
#define TR_BEEPCOUNTDOWN       TR(INDENT"Cta. atr\203s", INDENT"Cuenta atr\203s")
#define TR_PERSISTENT          TR(INDENT"Persisten.", INDENT"Persistente")
#define TR_BACKLIGHT_LABEL     "Luz fondo"
#define TR_GHOST_MENU_LABEL            "GHOST MENU"
#define TR_STATUS                      "Status"
#define TR_RAW_12BITS                  "Raw 12 bits"
#define TR_BLDELAY             INDENT"Duraci\211n"
#define TR_BLONBRIGHTNESS      INDENT"MAS brillo"
#define TR_BLOFFBRIGHTNESS     INDENT"MENOS brillo"
#define TR_KEYS_BACKLIGHT              "Keys backlight"
#define TR_BLCOLOR             INDENT "Color"
#define TR_SPLASHSCREEN        "Pantalla inicio"
#define TR_PWR_ON_DELAY        TR("Atraso On", "Atraso encendido")
#define TR_PWR_OFF_DELAY       TR("Atraso Off", "Atraso apagado")
#define TR_THROTTLEWARNING     TR("Aviso-A", INDENT "Aviso acelerador")
#define TR_SWITCHWARNING       TR("Aviso-I", INDENT "Aviso interruptor")
#define TR_POTWARNINGSTATE     TR(INDENT "Pot&Slid.", INDENT "Pots & sliders")
#define TR_SLIDERWARNING       TR(INDENT "Pos. slid.", INDENT "Posiciones slider")
#define TR_POTWARNING          TR("Aviso pot", INDENT "Aviso pot")
#define TR_TIMEZONE            TR("Zona horaria", INDENT "Zona horaria")
#define TR_ADJUST_RTC          TR("Ajustar RTC", INDENT "Ajustar RTC")
#define TR_GPS                 "GPS"
#define TR_RXCHANNELORD        TR("Rx Orden canal", "Orden habitual canales")
#define TR_STICKS              "Sticks"
#define TR_POTS                "Pots"
#define TR_SWITCHES            "Switches"
#define TR_SWITCHES_DELAY      TR("Atraso switch.", "Atraso switches")
#define TR_SLAVE               "Esclavo"
#define TR_MODESRC             "Modo\006% Fuente"
#define TR_MULTIPLIER          "Multiplicar"
#define TR_CAL                 "Cal"
#define TR_VTRIM               "Trim- +"
#define TR_BG                  "BG:"
#if defined(PCBHORUS)
  #define TR_MENUTOSTART       "Presiona [Enter] para empezar"
  #define TR_SETMIDPOINT       "Centra sticks/pots/sliders y presiona [Enter]"
  #define TR_MOVESTICKSPOTS    "Mueve sticks, pots and sliders y presiona [Enter]"
#else
  #define TR_MENUTOSTART       CENTER "\010" TR_ENTER " EMPEZAR"
  #define TR_SETMIDPOINT       TR(CENTER "\007STICKS AL CENTRO",CENTER "\010STICKS AL CENTRO")
  #define TR_MOVESTICKSPOTS    CENTER "\006MOVER STICKS/POTS"
#endif
#define TR_RXBATT              "Rx Bat.:"
#define TR_TXnRX               "Tx:\0Rx:"
#define OFS_RX                 4
#define TR_ACCEL               "Acc:"
#define TR_NODATA              CENTER "SIN DATOS"
#define TR_US                         "us"
#define TR_TMIXMAXMS           "Tmix m\203x"
#define TR_FREE_STACK          "Stack libre"
#define TR_MENUTORESET         TR_ENTER "Resetear"
#define TR_PPM_TRAINER         "TR"
#define TR_CH                  "CH"
#define TR_MODEL               "MODELO"
#define TR_FM                  "FM"
#define TR_MIX                 "MIX"
#define TR_EEPROMLOWMEM        "EEPROM mem.baja"
#define TR_ALERT               "\016ALERTA"
#define TR_PRESSANYKEYTOSKIP   "Pulsar tecla para omitir"
#define TR_THROTTLENOTIDLE     "Aceler. Activado"
#define TR_ALARMSDISABLED      "Alarmas Desact."
#define TR_PRESSANYKEY         TR("\010Pulsa una tecla", "Pulsa una tecla")
#define TR_BADEEPROMDATA       "Error datos EEPROM"
#define TR_BAD_RADIO_DATA      "Error datos radio"
#define TR_EEPROMFORMATTING    "Formateo EEPROM"
#define TR_STORAGE_FORMAT      "Preparaci\211n alamacenamiento"
#define TR_EEPROMOVERFLOW      "Desborde EEPROM"
#define TR_MENURADIOSETUP      "CONFIGURACI\210N"
#define TR_MENUDATEANDTIME     "FECHA Y HORA"
#define TR_MENUTRAINER         "ENTRENADOR"
#define TR_MENUSPECIALFUNCS    "FUNCIONES GLOBALES"
#define TR_MENUVERSION         "VERSI\210N"
#define TR_MENU_RADIO_SWITCHES TR("INTERUPTS", "TEST INTERRUPTORES")
#define TR_MENU_RADIO_ANALOGS  TR("ANAL\210GICOS", "TEST ANAL\210GICOS")
#define TR_MENUCALIBRATION     "CALIBRACI\210N"
#if defined(COLORLCD)
  #define TR_TRIMS2OFFSETS       "Trims => Offsets"
#else
  #define TR_TRIMS2OFFSETS       "\006Trims => Offsets"
#endif
#define TR_CHANNELS2FAILSAFE   "Canales=>Failsafe"
#define TR_CHANNEL2FAILSAFE    "Canal=>Failsafe"
#define TR_MENUMODELSEL        TR("MODELOS", "SELECCI\210N MODELO")
#define TR_MENUSETUP           TR("CONFIG. MODELO", "CONFIGURACI\210N MODELO")
#define TR_MENUFLIGHTMODE      "MODO DE VUELO"
#define TR_MENUFLIGHTMODES     "MODOS DE VUELO"
#define TR_MENUHELISETUP       "CONFIGURACI\210N HELI"


// Alignment

#if defined(PPM_CENTER_ADJUSTABLE) || defined(PPM_LIMITS_SYMETRICAL) // The right menu titles for the gurus ...
  #define TR_MENUINPUTS        "STICKS"
  #define TR_MENULIMITS        "SERVOS"
#else
  #define TR_MENUINPUTS        "DR/EXPO"
  #define TR_MENULIMITS        "L\206MITES"
#endif

#define TR_MENUCURVES          "CURVAS"
#define TR_MENUCURVE           "CURVA"
#define TR_MENULOGICALSWITCH   "INTERRUP.L\210GICO"
#define TR_MENULOGICALSWITCHES TR3("INTERRUP. L\210GICOS", "INTERRUP. L\210GICOS", "INTERRUPTORES L\210GICOS")
#define TR_MENUCUSTOMFUNC      TR("FUNCIONES", "FUNCIONES ESPECIALES")
#define TR_MENUCUSTOMSCRIPTS   "CUSTOM SCRIPTS"
#define TR_MENUTELEMETRY       "TELEMETR\206A"
#define TR_MENUTEMPLATES       "PLANTILLAS"
#define TR_MENUSTAT            TR("ESTAD.", "ESTAD\206STICAS")
#define TR_MENUDEBUG           "DEBUG"
#define TR_MONITOR_CHANNELS1   "MONITOR CANALES 1/8"
#define TR_MONITOR_CHANNELS2   "MONITOR CANALES 9/16"
#define TR_MONITOR_SWITCHES    "MONITOR INTERRP L\210GICOS"
#define TR_MONITOR_CHANNELS3   "MONITOR CANALES 17/24"
#define TR_MONITOR_CHANNELS4   "MONITOR CANALES 25/32"
#define TR_MONITOR_OUTPUT_DESC "SALIDAS"
#define TR_MONITOR_MIXER_DESC          "Mixers"
#define TR_RECEIVER_NUM                TR("Num Rx", "Receptor No.")
#define TR_RECEIVER                    "Receptor"
#define TR_MULTI_RFTUNE                TR("Sint.freq.", "RF Freq. sint.fina")
#define TR_MULTI_RFPOWER               "RF power"
#define TR_MULTI_WBUS                  "Output"
#define TR_MULTI_TELEMETRY             "Telemetr\207a"
#define TR_MULTI_VIDFREQ               TR("Freq.v\207deo", "Frecuencia v\207deo")
#define TR_RFPOWER                     "RF Power"
#define TR_MULTI_FIXEDID               TR("ID Fijo", "ID Fijo")
#define TR_MULTI_OPTION                TR("Opci\211n", "Valor opci\211n")
#define TR_MULTI_AUTOBIND              TR(INDENT "Emp Cnl",INDENT "Emparejar en canal")
#define TR_DISABLE_CH_MAP              TR("No ch map", "Desactivar mapa cnl")
#define TR_DISABLE_TELEM               TR("No telem", "Desactivar telem.")
#define TR_MULTI_DSM_AUTODTECT         TR(INDENT "Autodetect", INDENT "Autodetectar formato")
#define TR_MULTI_LOWPOWER              TR(INDENT "Baja poten.", INDENT "Modo de baja poten.")
#define TR_MULTI_LNA_DISABLE           INDENT "LNA desact."
#define TR_MODULE_TELEMETRY            TR(INDENT "S.Port", INDENT "S.Port link")
#define TR_MODULE_TELEM_ON             TR("ON", "Activado")
#define TR_DISABLE_INTERNAL            TR("Desac.RF int", "Desact. m\211d. int. RF")
#define TR_MODULE_NO_SERIAL_MODE       TR("No modo serie", "No en modo serie")
#define TR_MODULE_NO_INPUT             "Sin se\201al"
#define TR_MODULE_NO_TELEMETRY         TR3( "No telemetr\207a", "No MULTI_TELEMETRY", "No telemetr\207a (activa MULTI_TELEMETRY)")
#define TR_MODULE_WAITFORBIND          "Emparejar con protocolo"
#define TR_MODULE_BINDING              "Emparejando"
#define TR_MODULE_UPGRADE_ALERT        TR3("Actualizar", "Actualizar m\211dulo", "Actualizar\036m\211dulo")
#define TR_MODULE_UPGRADE              TR("Act recom", "Actualizar m\211dulo recomendado")
#define TR_PULSE_RATE                  "Pulse rate"
#define TR_LINK_SPEED                  "Link speed"
#define TR_REBIND                      "Reemparejar "
#define TR_REG_OK                      "Registro ok"
#define TR_BIND_OK                     "Emparejado ok"
#define TR_BINDING_CH1_8_TELEM_ON      "Ch1-8 Telem ON"
#define TR_BINDING_CH1_8_TELEM_OFF     "Ch1-8 Telem OFF"
#define TR_BINDING_CH9_16_TELEM_ON     "Ch9-16 Telem ON"
#define TR_BINDING_CH9_16_TELEM_OFF    "Ch9-16 Telem OFF"
#define TR_PROTOCOL_INVALID            TR("Prot. inv\203lido", "Protocolo inv\203lido")
#define TR_MODULE_STATUS               TR(INDENT "Estado", INDENT "Estado m\211dulo")
#define TR_MODULE_SYNC                 TR(INDENT "Sync", INDENT "Estado proto sync")
#define TR_MULTI_SERVOFREQ             TR("V ref serv", "Vel. refr. servo")
#define TR_MULTI_MAX_THROW             TR("M\203x. Throw", "Activar m\203x. throw")
#define TR_MULTI_RFCHAN                TR("Canal RF", "Selecciona canal RF")
#define TR_SYNCMENU                    "Sync " TR_ENTER
#define TR_LIMIT                       INDENT"L\207mite"
#define TR_MINRSSI                     "Min Rssi"
#define TR_LATITUDE                    "Latitud"
#define TR_LONGITUDE                   "Longitud"
#define TR_GPSCOORD                    TR("Coords GPS", INDENT "Formato coordenadas")
#define TR_VARIO                       TR("Vario", "Vari\211metro")
#define TR_PITCH_AT_ZERO               INDENT "Pitch en cero"
#define TR_PITCH_AT_MAX                INDENT "Pitch en m\203x"
#define TR_REPEAT_AT_ZERO              TR(INDENT "Repet. en cero", INDENT "Repetir en cero")
#define TR_SHUTDOWN                    "APAGANDO"
#define TR_SAVEMODEL                   "Grabando modelo"
#define TR_BATT_CALIB                  TR("Calib.bat.", "Calibraci\211n bater\207a")
#define TR_CURRENT_CALIB               "Calib. actual"
#define TR_VOLTAGE                     INDENT"Voltaje"
#define TR_CURRENT                     INDENT"Actual"
#define TR_SELECT_MODEL                TR("Selec. modelo", "Seleccionar modelo")
#define TR_SELECT_MODE                 "Select mode"
#define TR_CREATE_CATEGORY             "Crear categor\207a"
#define TR_RENAME_CATEGORY             "Renombrar categor\207a"
#define TR_DELETE_CATEGORY             "Borrar categor\207a"
#define TR_CREATE_MODEL                "Crear modelo"
#define TR_DUPLICATE_MODEL             "Duplicar modelo"
#define TR_COPY_MODEL                  "Copiar modelo"
#define TR_MOVE_MODEL                  "Mover modelo"
#define TR_BACKUP_MODEL                TR("Copia sgdad. mod.", "Copia seguridad modelo")
#define TR_DELETE_MODEL                "Borrar modelo"
#define TR_RESTORE_MODEL               "Restaurar modelo"
#define TR_DELETE_ERROR                "Error de borrado"
#define TR_CAT_NOT_EMPTY               "Categor\207a no esta vac\207a"
#define TR_SDCARD_ERROR                "Error SDCARD"
#define TR_NO_SDCARD                   "No SDCARD"
#define TR_WAITING_FOR_RX              "Esperando a RX..."
#define TR_WAITING_FOR_TX              "Esperando a TX..."
#define TR_WAITING_FOR_MODULE          TR("Waiting module", "Waiting for module...")
#define TR_NO_TOOLS                    "No hay utils"
#define TR_NORMAL                      "Normal"
#define TR_NOT_INVERTED                "No inv"
#define TR_NOT_CONNECTED               "!Conectado"
#define TR_CONNECTED                   "Conectado"
#define TR_FLEX_915                    "Flex 915MHz"
#define TR_FLEX_868                    "Flex 868MHz"
#define TR_16CH_WITHOUT_TELEMETRY      TR("16CH sin telem.", "16CH sin telemetr\207a")
#define TR_16CH_WITH_TELEMETRY         TR("16CH con telem.", "16CH con telemetr\207a")
#define TR_8CH_WITH_TELEMETRY          TR("8CH con telem.", "8CH con telemetr\207a")
#define TR_EXT_ANTENNA                 "Antena ext."
#define TR_PIN                         "Pin"
#define TR_UPDATE_RX_OPTIONS           "Actualizar opciones RX?"
#define TR_UPDATE_TX_OPTIONS           "Actualizar opciones TX?"
#define TR_MODULES_RX_VERSION          "M\211dulos / versi\211n RX"
#define TR_MENU_MODULES_RX_VERSION     "M\210DULOS / VERSI\210N RX"
#define TR_MENU_FIRM_OPTIONS           "OPCIONES FIRMWARE"
#define TR_GYRO                        "Gyro"
#define TR_STICKS_POTS_SLIDERS         "Sticks/Pots/Sliders"
#define TR_PWM_STICKS_POTS_SLIDERS     "PWM Sticks/Pots/Sliders"
#define TR_RF_PROTOCOL                 "Protocolo RF"
#define TR_MODULE_OPTIONS              "Opciones m\211dulo"
#define TR_POWER                       "Potencia"
#define TR_NO_TX_OPTIONS               "Sin opciones TX"
#define TR_RTC_BATT                    TR("Bat. RTC", "Bater\207a RTC")
#define TR_POWER_METER_EXT             "Medidor potencia(EXT)"
#define TR_POWER_METER_INT             "Medidor potencia(INT)"
#define TR_SPECTRUM_ANALYSER_EXT       "Espectro (EXT)"
#define TR_SPECTRUM_ANALYSER_INT       "Espectro (INT)"
#define TR_SDCARD_FULL                 "SD Card llena"
#define TR_NEEDS_FILE                  "NECESITA ARCHIVO"
#define TR_EXT_MULTI_SPEC              "opentx-inv"
#define TR_INT_MULTI_SPEC              "stm-opentx-noinv"
#define TR_INCOMPATIBLE        "Incompatible"
#define TR_WARNING             "AVISO"
#define TR_EEPROMWARN          "EEPROM"
#define TR_STORAGE_WARNING     "ALMAC."
#define TR_EEPROM_CONVERTING   "Convirtiendo EEPROM"
#define TR_THROTTLEWARN        TR("ACELERAD.", "ACELERADOR")
#define TR_ALARMSWARN          "ALARMAS"
#define TR_SWITCHWARN          TR("INTERRUP.", "INTERRUPTOR")
#define TR_FAILSAFEWARN        "FAILSAFE"
#define TR_TEST_WARNING         TR("TESTING", "TEST BUILD")
#define TR_TEST_NOTSAFE         "Usar solo para test"
#define TR_WRONG_SDCARDVERSION  TR("Ver esperada: ", "Versi\211n esperada: ")
#define TR_WARN_RTC_BATTERY_LOW "Bater\207a RTC baja"
#define TR_WARN_MULTI_LOWPOWER  "Modo baja potencia"
#define TR_BATTERY              "BATER\206A"
#define TR_WRONG_PCBREV        "Placa PCB err\211nea"
#define TR_EMERGENCY_MODE      "MODO EMERGENCIA"
#define TR_PCBREV_ERROR        "Error PCB"
#define TR_NO_FAILSAFE         "Failsafe no fijado"
#define TR_KEYSTUCK            "Tecla atascada"
#define TR_INVERT_THR          TR("Invertir acel?", "Invertir acel.?")
#define TR_SPEAKER_VOLUME      INDENT "Volumen"
#define TR_LCD                 "LCD"
#define TR_BRIGHTNESS          INDENT "Brillo"
#define TR_CPU_TEMP            "CPU Temp.\016>"
#define TR_CPU_CURRENT         "Actual\022>"
#define TR_CPU_MAH             "Consumo"
#define TR_COPROC              "CoProc."
#define TR_COPROC_TEMP         "MB Temp. \016>"
#define TR_CAPAWARNING         INDENT "Capacidad baja"
#define TR_TEMPWARNING         INDENT "Sobrecalent"
#define TR_FUNC                "Funci\211n"
#define TR_V1                  "V1"
#define TR_V2                  "V2"
#define TR_DURATION            "Duraci\211n"
#define TR_DELAY               "Atraso"
#define TR_SD_CARD             "SD CARD"
#define TR_SDHC_CARD           "SD-HC CARD"
#define TR_NO_SOUNDS_ON_SD     "Sin sonidos en SD"
#define TR_NO_MODELS_ON_SD     "Sin modelos en SD"
#define TR_NO_BITMAPS_ON_SD    "Sin im\203genes en SD"
#define TR_NO_SCRIPTS_ON_SD    "No scripts en SD"
#define TR_SCRIPT_SYNTAX_ERROR TR("Syntax error", "Script syntax error")
#define TR_SCRIPT_PANIC        "Script panic"
#define TR_SCRIPT_KILLED       "Script killed"
#define TR_SCRIPT_ERROR        "Error desconocido"
#define TR_PLAY_FILE           "Play"
#define TR_DELETE_FILE         "Borrar"
#define TR_COPY_FILE           "Copiar"
#define TR_RENAME_FILE         "Renombrar"
#define TR_ASSIGN_BITMAP       "Asignar imagen"
#define TR_ASSIGN_SPLASH       "Pant. bienvenida"
#define TR_EXECUTE_FILE        "Ejecutar"
#define TR_REMOVED             "Borrado"
#define TR_SD_INFO             "Informaci\211n"
#define TR_SD_FORMAT           "Formatear"
#define TR_NA                  "N/A"
#define TR_HARDWARE            "COMPONENTES"
#define TR_FORMATTING          "Formateando.."
#define TR_TEMP_CALIB          "Temp. Calib"
#define TR_TIME                "Hora"
#define TR_MAXBAUDRATE         "M\203x bauds"

#define TR_BLUETOOTH            "Bluetooth"
#define TR_BLUETOOTH_DISC       "Buscar"
#define TR_BLUETOOTH_INIT       "Init"
#define TR_BLUETOOTH_DIST_ADDR  "Dir. remota"
#define TR_BLUETOOTH_LOCAL_ADDR "Dir. local"
#define TR_BLUETOOTH_PIN_CODE   "C\211digo PIN"
#define TR_BAUDRATE             "Velocidad BT"
#define LEN_BLUETOOTH_MODES     "\012"
#if defined(PCBX9E)
#define TR_BLUETOOTH_MODES      "---\0      ""Activo\0   "
#else
#define TR_BLUETOOTH_MODES      "---\0      ""Telemetr\207a""Entrenador"
#endif
#define TR_SD_INFO_TITLE       "SD INFO"
#define TR_SD_TYPE             "Tipo:"
#define TR_SD_SPEED            "Velocidad:"
#define TR_SD_SECTORS          "Sectores:"
#define TR_SD_SIZE             "Tama\201o:"
#define TR_TYPE                INDENT "Tipo"
#define TR_GLOBAL_VARS         "Variables globales"
#define TR_GVARS               "V. GLOBAL"
#define TR_GLOBAL_VAR                  "Variable global"
#define TR_MENUGLOBALVARS              "VARIABLES GLOBALES"
#define TR_OWN                         "Propio"
#define TR_DATE                        "Fecha"
#define TR_MONTHS                      { "Ene", "Feb", "Mar", "Abr", "May", "Jun", "Jul", "Ago", "Sep", "Oct", "Nov", "Dic" }
#define TR_ROTARY_ENCODER              "R.E."
#define TR_INVERT_ROTARY               "Invert Rotary"
#define TR_CHANNELS_MONITOR            "MONITOR CANALES"
#define TR_MIXERS_MONITOR              "MONITOR MEZCLAS"
#define TR_PATH_TOO_LONG               "Path muy largo"
#define TR_VIEW_TEXT                   "Ver texto"
#define TR_FLASH_BOOTLOADER            "Flash bootloader"
#define TR_FLASH_EXTERNAL_DEVICE       "Flash disp. externo"
#define TR_FLASH_RECEIVER_OTA          "Flash RX OTA"
#define TR_FLASH_RECEIVER_BY_EXTERNAL_MODULE_OTA "Flash RX por ext. OTA"
#define TR_FLASH_RECEIVER_BY_INTERNAL_MODULE_OTA "Flash RX por int. OTA"
#define TR_FLASH_FLIGHT_CONTROLLER_BY_EXTERNAL_MODULE_OTA "Flash FC by ext. OTA"
#define TR_FLASH_FLIGHT_CONTROLLER_BY_INTERNAL_MODULE_OTA "Flash FC by int. OTA"
#define TR_FLASH_BLUETOOTH_MODULE      TR("Flash m\211dulo BT", "Flash m\211dulo bluetooth")
#define TR_FLASH_POWER_MANAGEMENT_UNIT "Flash unid. pwr mngt"
#define TR_CURRENT_VERSION             TR("Vers. actual", "Versi\211n actual: ")
#define TR_HW_REV                      "HW Rev"
#define TR_FLASH_INTERNAL_MODULE       TR("Flash m\211dulo int", "Flash m\211dulo interno")
#define TR_FLASH_INTERNAL_MULTI        TR("Flash Multi int", "Flash Multi interno")
#define TR_FLASH_EXTERNAL_MODULE       TR("Flash m\211dulo ext", "Flash m\211dulo externo")
#define TR_FLASH_EXTERNAL_MULTI        TR("Flash Multi ext", "Flash Multi externo")
#define TR_FLASH_EXTERNAL_ELRS         TR("Flash Ext. ELRS", "Flash External ELRS")
#define TR_FIRMWARE_UPDATE_ERROR       TR("Error act FW", "Error actualiz. firmware")
#define TR_FIRMWARE_UPDATE_SUCCESS     "Flash ok"
#define TR_WRITING                     "Escribiendo..."
#define TR_CONFIRM_FORMAT              "Confirmar formato?"
#define TR_INTERNALRF                  "RF interna"
#define TR_INTERNAL_MODULE             TR("M\211dulo int", "M\211dulo interno")
#define TR_EXTERNAL_MODULE             TR("M\211dulo ext", "M\211dulo externo")
#define TR_OPENTX_UPGRADE_REQUIRED     "Actualiz. OpenTX requerida"
#define TR_TELEMETRY_DISABLED          "Telem. inactiva"
#define TR_MORE_OPTIONS_AVAILABLE      "M\203s opciones disp."
#define TR_NO_MODULE_INFORMATION       "Sin informaci\211n m\211dulo"
#define TR_EXTERNALRF                  "RF externa"
#define TR_FAILSAFE                    INDENT"Failsafe"
#define TR_FAILSAFESET                 "AJUSTES FAILSAFE"
#define TR_REG_ID                      "Reg. ID"
#define TR_OWNER_ID                    "Pers. ID"
#define TR_PINMAPSET                   "PINMAP"
#define TR_HOLD                        "Hold"
#define TR_HOLD_UPPERCASE              "HOLD"
#define TR_NONE                        "Nada"
#define TR_NONE_UPPERCASE              "NADA"
#define TR_MENUSENSOR                  "SENSOR"
#define TR_POWERMETER_PEAK             "Pico"
#define TR_POWERMETER_POWER            "Potencia"
#define TR_POWERMETER_ATTN             "Attn"
#define TR_POWERMETER_FREQ             "Frec."
#define TR_MENUTOOLS                   "UTILS"
#define TR_TURN_OFF_RECEIVER           "Apaga el receptor"
#define TR_STOPPING                    "Parando..."
#define TR_MENU_SPECTRUM_ANALYSER      "ANALIZADOR DE ESPECTRO"
#define TR_MENU_POWER_METER            "MEDIDOR POTENCIA"
#define TR_SENSOR                      "SENSOR"
#define TR_COUNTRYCODE                 "C\211digo pa\207s"
#define TR_USBMODE                     "Modo USB"
#define TR_JACKMODE                    "Modo Jack"
#define TR_VOICELANG                   "Idioma voces"
#define TR_UNITSSYSTEM                 "Unidades"
#define TR_EDIT                        "Editar"
#define TR_INSERT_BEFORE               "Insertar antes"
#define TR_INSERT_AFTER                "Insertar despu\205s"
#define TR_COPY                        "Copiar"
#define TR_MOVE                "Mover"
#define TR_PASTE               "Pegar"
#define TR_DELETE              "Borrar"
#define TR_INSERT              "Insertar"
#define TR_RESET_FLIGHT        "Reset Vuelo"
#define TR_RESET_TIMER1        "Reset Reloj 1"
#define TR_RESET_TIMER2        "Reset Reloj 2"
#define TR_RESET_TIMER3        "Reset Reloj 3"
#define TR_RESET_TELEMETRY     "Reset Telemetr\207a"
#define TR_STATISTICS          "Estad\207sticas"
#define TR_ABOUT_US            "Nosotros"
#define TR_USB_JOYSTICK        "Joystick USB (HID)"
#define TR_USB_MASS_STORAGE    "Almaz. USB (SD)"
#define TR_USB_SERIAL          "Serie USB (Debug)"
#define TR_USB_TELEMETRY       "USB Telem mirror"
#define TR_SETUP_SCREENS       "Pantallas config"
#define TR_MONITOR_SCREENS     "Monitores"
#define TR_AND_SWITCH          TR("Inter. AND", "Interruptor AND")
#define TR_SF                  "CF"
#define TR_GF                  "GF"
#define TR_SPEAKER             INDENT"Altavoz"
#define TR_BUZZER              INDENT"Zumbador"
#define TR_BYTES               "bytes"
#define TR_MODULE_BIND         "[Enl.]"
#define TR_POWERMETER_ATTN_NEEDED      "Necesita atenuador"
#define TR_PXX2_SELECT_RX              "Selecciona RX..."
#define TR_PXX2_DEFAULT                "<defecto>"
#define TR_BT_SELECT_DEVICE            "Selecciona dispositivo"
#define TR_DISCOVER             "Buscar"
#define TR_BUTTON_INIT                 BUTTON("Init")
#define TR_WAITING                     "Espera..."
#define TR_RECEIVER_DELETE             "Borrar receptor?"
#define TR_RECEIVER_RESET              "Reset receptor?"
#define TR_SHARE                       "Compartido"
#define TR_BIND                        "Emparejar"
#define TR_REGISTER             TR("Reg", "Registrar")
#define TR_MODULE_RANGE        "[Lim.]"
#define TR_RECEIVER_OPTIONS            TR("REC. OPTIONS", "OPCIONES RECEPTOR")
#define TR_DEL_BUTTON                  BUTTON(TR("Del", "Borrar"))
#define TR_RESET_BTN           "[Reset]"
#define TR_DEBUG                       "Debug"
#define TR_KEYS_BTN                BUTTON(TR("SW","Switches"))
#define TR_ANALOGS_BTN                 BUTTON(TR("Analog","Analogs"))
#define TR_TOUCH_NOTFOUND              "Hardware t\203ctil no encontrado"
#define TR_TOUCH_EXIT                  "Tocar pantalla para salir"
#define TR_CALIBRATION                   "Calibraci\211n"
#define TR_SET                 "[Ajuste]"
#define TR_TRAINER             "Entrenador"
#define TR_CHANS                       "Chans"
#define TR_ANTENNAPROBLEM      CENTER "Problema antena TX!"
#define TR_MODELIDUSED         TR("ID en uso", "ID modelo en uso")
#define TR_MODULE              "M\211dulo"
#define TR_RX_NAME                     "Nombre Rx"
#define TR_TELEMETRY_TYPE      TR("Tipo", "Tipo telemetr\207a")
#define TR_TELEMETRY_SENSORS   "Sensores"
#define TR_VALUE               "Valor"
#define TR_TOPLCDTIMER         "Timer LCD superior"
#define TR_UNIT                "Unidad"
#define TR_TELEMETRY_NEWSENSOR INDENT "A\201adir sensor..."
#define TR_CHANNELRANGE        INDENT "Canales"
#define TR_AFHDS3_RX_FREQ              TR("RX freq.", "RX frequencia")
#define TR_AFHDS3_ONE_TO_ONE_TELEMETRY TR("Unicast/Tel.", "Unicast/Telemetr\207a")
#define TR_AFHDS3_ONE_TO_MANY          "Multicast"
#define TR_AFHDS3_ACTUAL_POWER         TR("Act. pow", "Actual power")
#define TR_AFHDS3_POWER_SOURCE         TR("Power src.", "Power source")
#define TR_ANTENNACONFIRM1     "ANTENA EXT."
#if defined(PCBX12S)
#define LEN_ANTENNA_MODES      "\020"
#define TR_ANTENNA_MODES       "Interna\0        ""Preguntar\0      ""Por modelo\0     ""Interno + Externo\0"
#else
#define LEN_ANTENNA_MODES      "\012"
#define TR_ANTENNA_MODES       "Interna\0  ""Preguntar\0""Por modelo""Externa\0  "
#endif
#define TR_USE_INTERNAL_ANTENNA        TR("Usa antena int.", "Usa antena interna")
#define TR_USE_EXTERNAL_ANTENNA        TR("Use antena ext.", "Usa antena externa")
#define TR_ANTENNACONFIRM2     TR("Revisa antena", "Revisa que la antena est\205 instalada!")

#define TR_MODULE_PROTOCOL_FLEX_WARN_LINE1        "No requerido"
#define TR_MODULE_PROTOCOL_FCC_WARN_LINE1         "Requiere FCC"
#define TR_MODULE_PROTOCOL_EU_WARN_LINE1          "Requiere EU"
#define TR_MODULE_PROTOCOL_WARN_LINE2             "firmware certificado"
#define TR_LOWALARM            INDENT "Alarma baja"
#define TR_CRITICALALARM       INDENT "Alarma cr\207tica"
#define TR_RSSIALARM_WARN      TR("RSSI", "TELEMETRY RSSI")
#define TR_NO_RSSIALARM        TR(INDENT "Alarmas desact.", INDENT "Alarmas telemetr\207a desact.")
#define TR_DISABLE_ALARM       TR(INDENT "Desact. alarmas", INDENT "Desact. alarmas telem.")
#define TR_ENABLE_POPUP        "Activa Popup"
#define TR_DISABLE_POPUP       "Desactiva Popup"
#define TR_POPUP               "Popup"
#define TR_MIN                 "Min"
#define TR_MAX                 "M\203x"
#define TR_CURVE_PRESET        "Preset..."
#define TR_PRESET              "Preset"
#define TR_MIRROR              "Mirror"
#define TR_CLEAR               "Borrar"
#define TR_RESET               "Reset"
#define TR_RESET_SUBMENU       "Reset..."
#define TR_COUNT               "Puntos"
#define TR_PT                  "pt"
#define TR_PTS                 "pts"
#define TR_SMOOTH              TR3("Suaviz.", "Suaviz.", "Suavizado")
#define TR_COPY_STICKS_TO_OFS  TR("Copia stick->subtr", "Copia Sticks a Offset")
#define TR_COPY_MIN_MAX_TO_OUTPUTS     TR("Copia min/m\203x a sal",  "Copia min/m\203x/ctr a salidas")
#define TR_COPY_TRIMS_TO_OFS   TR("Copia trim->subtr", "Copia trims a subtrim")
#define TR_INCDEC              "Inc/Decrement"
#define TR_GLOBALVAR           "Var global"
#define TR_MIXSOURCE           "Entrada mixer"
#define TR_CONSTANT            "Constante"
#define TR_PERSISTENT_MAH      INDENT "Valor mAh"
#define TR_PREFLIGHT           "Chequeos prevuelo"
#define TR_CHECKLIST           TR(INDENT "Lista verif", INDENT "Lista verificaci\211n")
#define TR_FAS_OFFSET          TR(INDENT "FAS Ofs", INDENT "FAS Offset")
#define TR_AUX_SERIAL_MODE     "Puerto serie"
#define TR_AUX2_SERIAL_MODE    "Puerto serie 2"
#define TR_SCRIPT              "Script"
#define TR_INPUTS              "Entradas"
#define TR_OUTPUTS             "Salidas"
#define STR_EEBACKUP           "Copia seg. EEPROM"
#define STR_FACTORYRESET       "Rest. fabrica"
#define TR_CONFIRMRESET        "Borrar TODOS los " LCDW_128_480_LINEBREAK "modelos y conf?"
#define TR_TOO_MANY_LUA_SCRIPTS "Demasiados Lua scripts!"
#define TR_SPORT_UPDATE_POWER_MODE     "SP Power"
#define LEN_SPORT_UPDATE_POWER_MODES   "\004"
#define TR_SPORT_UPDATE_POWER_MODES    "AUTO""ON\0 "
#define TR_NO_TELEMETRY_SCREENS        TR("No hay pant. telemetr\207a", "No hay pantallas de telemetr\207a")
#define TR_TOUCH_PANEL                 "Pantalla t\203ctil:"

// Horus and Taranis column headers
#define TR_PHASES_HEADERS_NAME         "Nombre"
#define TR_PHASES_HEADERS_SW           "Interrup"
#define TR_PHASES_HEADERS_RUD_TRIM     "Trim timon"
#define TR_PHASES_HEADERS_ELE_TRIM     "Trim elevador"
#define TR_PHASES_HEADERS_THT_TRIM     "Trim potencia"
#define TR_PHASES_HEADERS_AIL_TRIM     "Trim aler\211n"
#define TR_PHASES_HEADERS_CH5_TRIM     "Trim 5"
#define TR_PHASES_HEADERS_CH6_TRIM     "Trim 6"
#define TR_PHASES_HEADERS_FAD_IN       "Aparecer"
#define TR_PHASES_HEADERS_FAD_OUT      "Desparecer"

#define TR_LIMITS_HEADERS_NAME         "Nombre"
#define TR_LIMITS_HEADERS_SUBTRIM      "Compensaci\211n"
#define TR_LIMITS_HEADERS_MIN          "Min"
#define TR_LIMITS_HEADERS_MAX          "M\203x"
#define TR_LIMITS_HEADERS_DIRECTION    "Direcci\211n"
#define TR_LIMITS_HEADERS_CURVE        "Curva"
#define TR_LIMITS_HEADERS_PPMCENTER    "Centrado PPM"
#define TR_LIMITS_HEADERS_SUBTRIMMODE  "Sim\205trica"

#define TR_LSW_HEADERS_FUNCTION        "Funci\211n"
#define TR_LSW_HEADERS_V1              "V1"
#define TR_LSW_HEADERS_V2              "V2"
#define TR_LSW_HEADERS_ANDSW           "Interrup. AND"
#define TR_LSW_HEADERS_DURATION        "Duraci\211n"
#define TR_LSW_HEADERS_DELAY           "Retardo"

#define TR_GVAR_HEADERS_NAME          "Nombre"
#define TR_GVAR_HEADERS_FM0           "Valor en FM0"
#define TR_GVAR_HEADERS_FM1           "Valor en FM1"
#define TR_GVAR_HEADERS_FM2           "Valor en FM2"
#define TR_GVAR_HEADERS_FM3           "Valor en FM3"
#define TR_GVAR_HEADERS_FM4           "Valor en FM4"
#define TR_GVAR_HEADERS_FM5           "Valor en FM5"
#define TR_GVAR_HEADERS_FM6           "Valor en FM6"
#define TR_GVAR_HEADERS_FM7           "Valor en FM7"
#define TR_GVAR_HEADERS_FM8           "Valor en FM8"

// Horus footer descriptions
#define TR_LSW_DESCRIPTIONS    { "Tipo de comparaci\211n o funci\211n", "Primera variable", "Segunda variable o constante", "Segunda variable o constante", "Condici\211n adicional para activar l\207nea", "Duraci\211n m\207nima de switch l\211gico", "Duraci\211n m\207nima TRUE para activar switch l\211gico" }

// Horus layouts and widgets
#define TR_FIRST_CHANNEL              "Primer canal"
#define TR_FILL_BACKGROUND            "Rellenar fondo"
#define TR_BG_COLOR                   "Color BG"
#define TR_SLIDERS_TRIMS              "Sliders+Trims"
#define TR_SLIDERS                    "Sliders"
#define TR_FLIGHT_MODE                "Modo de vuelo"
#define TR_INVALID_FILE               "Fichero no valido"
#define TR_TIMER_SOURCE               "Entrada timer"
#define TR_SIZE                       "Tama\201o"
#define TR_SHADOW                     "Sombra"
#define TR_TEXT                       "Texto"
#define TR_COLOR                      "Color"
#define TR_MAIN_VIEW_X                "Vista principal X"
#define TR_PANEL1_BACKGROUND          "Fondo panel1"
#define TR_PANEL2_BACKGROUND          "Fondo panel2"

// Taranis About screen
#define TR_ABOUTUS             "Nosotros"

#define TR_ABOUT_OPENTX_1      "OpenTX es open source, no-"
#define TR_ABOUT_OPENTX_2      "comercial y viene sin "
#define TR_ABOUT_OPENTX_3      "garantias. Fue desarrollado"
#define TR_ABOUT_OPENTX_4      "libremente. Ayudas a trav\205s"
#define TR_ABOUT_OPENTX_5      "de donaciones son bienvenidas!"

#define TR_ABOUT_BERTRAND_1    "Bertrand Songis"
#define TR_ABOUT_BERTRAND_2    "Autor principal OpenTX"
#define TR_ABOUT_BERTRAND_3    "Companion codesarrollador"

#define TR_ABOUT_MIKE_1        "Mike Blandford"
#define TR_ABOUT_MIKE_2        "Gur\213 de codigo y drivers"
#define TR_ABOUT_MIKE_3        "Seguramente uno de los"
#define TR_ABOUT_MIKE_4        "mejores inspiradores"

#define TR_ABOUT_ROMOLO_1      "Romolo Manfredini"
#define TR_ABOUT_ROMOLO_2      "Desarrollador principal"
#define TR_ABOUT_ROMOLO_3      "Companion"

#define TR_ABOUT_ANDRE_1       "Andre Bernet"
#define TR_ABOUT_ANDRE_2       "Funcionalidad, usabilidad,"
#define TR_ABOUT_ANDRE_3       "depurado, documentaci\211n"

#define TR_ABOUT_ROB_1         "Rob Thomson"
#define TR_ABOUT_ROB_2         "openrcforums webmaster"

#define TR_ABOUT_KJELL_1       "Kjell Kernen"
#define TR_ABOUT_KJELL_2       "www.open-tx.org main author"
#define TR_ABOUT_KJELL_3       "Autor OpenTX Recorder"
#define TR_ABOUT_KJELL_4       "Colaborador Companion"

#define TR_ABOUT_MARTIN_1      "Martin Hotar"
#define TR_ABOUT_MARTIN_2      "Dise\201o gr\203fico"

#if defined(MANUFACTURER_FRSKY)
  #define TR_ABOUT_HARDWARE_1  "FrSky"
  #define TR_ABOUT_HARDWARE_2  "Productor dise\201o de hardware"
  #define TR_ABOUT_HARDWARE_3  "Colaborador firmware"
#elif defined(MANUFACTURER_RADIOMASTER)
  #define TR_ABOUT_HARDWARE_1  "Radiomaster"
  #define TR_ABOUT_HARDWARE_2  "Productor dise\201o de hardware"
  #define TR_ABOUT_HARDWARE_3  "Colaborador firmware"
#elif defined(MANUFACTURER_JUMPER)
  #define TR_ABOUT_HARDWARE_1  "JumperRC"
  #define TR_ABOUT_HARDWARE_2  "Productor dise\201o de hardware"
  #define TR_ABOUT_HARDWARE_3  "Colaborador firmware"
#else
  #define TR_ABOUT_HARDWARE_1  "Brent Nelson"
  #define TR_ABOUT_HARDWARE_2  "Sky9x dise\201ador/productor"
  #define TR_ABOUT_HARDWARE_3  ""
#endif

#define TR_ABOUT_PARENTS_1     "Padres del proyecto"
#define TR_ABOUT_PARENTS_2     "ersky9x (Mike Blandford)"
#define TR_ABOUT_PARENTS_3     "ER9X (Erez Raviv)"
#define TR_ABOUT_PARENTS_4     "TH9X (Thomas Husterer)"

#define TR_CHR_SHORT  's'
#define TR_CHR_LONG   'l'
#define TR_CHR_TOGGLE 't'
#define TR_CHR_HOUR   'h'
#define TR_CHR_INPUT  'I'   // Values between A-I will work

#define TR_BEEP_VOLUME         "Volumen Beep"
#define TR_WAV_VOLUME          "Volumen Wav"
#define TR_BG_VOLUME           "Volumen Bg"

#define TR_TOP_BAR             "Panel superior"
#define TR_FLASH_ERASE                 "Borrar flash..."
#define TR_FLASH_WRITE                 "Escribir flash..."
#define TR_OTA_UPDATE                  "Actualizaci\211n OTA..."
#define TR_MODULE_RESET                "Reset m\211dulo..."
#define TR_UNKNOWN_RX                  "RX desconocido"
#define TR_UNSUPPORTED_RX              "RX no soportado"
#define TR_OTA_UPDATE_ERROR            "Error actualizaci\211n OTA"
#define TR_DEVICE_RESET                "Reset dispositivo..."
#define TR_ALTITUDE            INDENT "Altitud"
#define TR_SCALE               "Escala"
#define TR_VIEW_CHANNELS       "Ver Canales"
#define TR_VIEW_NOTES          "Ver Notas"
#define TR_MODEL_SELECT        "Seleccionar modelo"
#define TR_MODS_FORBIDDEN      "Modificaciones prohibidas"
#define TR_UNLOCKED            "Desbloqueado"
#define TR_ID                  "ID"
#define TR_PRECISION           "Precisi\211n"
#define TR_RATIO               "Ratio"
#define TR_FORMULA             "F\211rmula"
#define TR_CELLINDEX           "Cell index"
#define TR_LOGS                "Logs"
#define TR_OPTIONS             "Opciones"
#define TR_FIRMWARE_OPTIONS    "Opciones firmware"

#define TR_ALTSENSOR           "Alt sensor"
#define TR_CELLSENSOR          "Cell sensor"
#define TR_GPSSENSOR           "GPS sensor"
#define TR_CURRENTSENSOR       "Sensor"
#define TR_AUTOOFFSET          "Auto offset"
#define TR_ONLYPOSITIVE        "Positivo"
#define TR_FILTER              "Filtro"
#define TR_TELEMETRYFULL       TR("Telem. llena!", "Todas las entradas de telemetr\207a llenas!")
#define TR_SERVOS_OK           "Servos OK"
#define TR_SERVOS_KO           "Servos KO"
#define TR_INVERTED_SERIAL     INDENT "Invertir"
#define TR_IGNORE_INSTANCE     TR(INDENT "No inst.", INDENT "Ignora instancias")
#define TR_DISCOVER_SENSORS    "Buscar sensores"
#define TR_STOP_DISCOVER_SENSORS "Parar busqueda"
#define TR_DELETE_ALL_SENSORS  "Borrar sensores"
#define TR_CONFIRMDELETE       "Seguro " LCDW_128_480_LINEBREAK "borrar todo ?"
#define TR_SELECT_WIDGET       "Seleccionar widget"
#define TR_REMOVE_WIDGET       "Borrar widget"
#define TR_WIDGET_SETTINGS     "Config. widget"
#define TR_REMOVE_SCREEN       "Borrar pantalla"
#define TR_SETUP_WIDGETS       "Config. widgets"
#define TR_USER_INTERFACE      "Interfaz"
#define TR_THEME               "Tema"
#define TR_SETUP               "Configuraci\211n"
#define TR_MAINVIEWX           "Vista pral. X"
#define TR_LAYOUT              "Dise\201o"
#define TR_ADDMAINVIEW         "A\201adir vista pral."
#define TR_BACKGROUND_COLOR    "Color de fondo"
#define TR_MAIN_COLOR          "Color principal"
#define TR_BAR2_COLOR                  "Color barra secundaria"
#define TR_BAR1_COLOR                  "Color barra principal"
#define TR_TEXT_COLOR                  "Color texto"
#define TR_TEXT_VIEWER         "Visor de texto"

#define TR_MENU_INPUTS         "\314Entradas"
#define TR_MENU_LUA            "\322Lua scripts"
#define TR_MENU_STICKS         "\307Sticks"
#define TR_MENU_POTS           "\310Pots"
#define TR_MENU_MAX            "\315MAX"
#define TR_MENU_HELI           "\316C\207clico"
#define TR_MENU_TRIMS          "\313Trims"
#define TR_MENU_SWITCHES       "\312Interruptores"
#define TR_MENU_LOGICAL_SWITCHES TR("\312Interr. l\211gicos", "\312Interruptores l\211gicos")
#define TR_MENU_TRAINER        "\317Entrenador"
#define TR_MENU_CHANNELS       "\320Canales"
#define TR_MENU_GVARS          "\311GVars"
#define TR_MENU_TELEMETRY      "\321Telemetr\207a"
#define TR_MENU_DISPLAY        "MONITOR"
#define TR_MENU_OTHER          "Otros"
#define TR_MENU_INVERT         "Invertir"
#define TR_JITTER_FILTER       "Filtro ADC"
#define TR_RTC_CHECK           TR("Check RTC", "Check RTC voltaje")
#define TR_AUTH_FAILURE        "Fallo " LCDW_128_480_LINEBREAK  "autentificaci\211n"
#define TR_RACING_MODE         "Racing mode"

#define ZSTR_VFR               "\026\006\022"
#define ZSTR_RSSI              "\022\023\023\011"
#define ZSTR_R9PW                      "\022\044\020\027"
#define ZSTR_RAS               "\023\027\022"
#define ZSTR_A1                "\001\034"
#define ZSTR_A2                "\001\035"
#define ZSTR_A3                "\001\036"
#define ZSTR_A4                "\001\037"
#define ZSTR_BATT              "\022\350\002\354"
#define ZSTR_ALT               "\001\364\354"
#define ZSTR_TEMP1             "\024\363\360\034"
#define ZSTR_TEMP2             "\024\363\360\035"
#define ZSTR_TEMP3                     "\024\363\360\036"
#define ZSTR_TEMP4                     "\024\363\360\037"
#define ZSTR_RPM2                      "\022\020\015\035"
#define ZSTR_PRES                      "\020\356\373\355"
#define ZSTR_ODO1                      "\017\374\361\034"
#define ZSTR_ODO2                      "\017\374\361\035"
#define ZSTR_TXV                       "\024\030_\026"
#define ZSTR_CURR_SERVO1               "\003\023\352\034"
#define ZSTR_CURR_SERVO2               "\003\023\352\035"
#define ZSTR_CURR_SERVO3               "\003\023\352\036"
#define ZSTR_CURR_SERVO4               "\003\023\352\037"
#define ZSTR_DIST                      "\004\367\355\354"
#define ZSTR_ARM                       "\001\356\363"
#define ZSTR_C50                       "\003\040\033"
#define ZSTR_C200                      "\003\035\033\033"
#define ZSTR_RPM               "\022\020\015"
#define ZSTR_FUEL              "\006\353\373\364"
#define ZSTR_VSPD              "\026\023\360\374"
#define ZSTR_ACCX              "\001\375\375\030"
#define ZSTR_ACCY              "\001\375\375\031"
#define ZSTR_ACCZ              "\001\375\375\032"
#define ZSTR_GYROX             "\007\031\022\030"
#define ZSTR_GYROY             "\007\031\022\031"
#define ZSTR_GYROZ             "\007\031\022\032"
#define ZSTR_CURR              "\003\353\356\356"
#define ZSTR_CAPACITY          "\003\377\360\377"
#define ZSTR_VFAS              "\026\006\001\023"
#define ZSTR_BATT_PERCENT      "\002\377\354%"
#define ZSTR_ASPD              "\001\023\360\374"
#define ZSTR_GSPD              "\007\023\360\374"
#define ZSTR_HDG               "\010\374\371"
#define ZSTR_SATELLITES        "\023\377\354\355"
#define ZSTR_CELLS             "\003\373\364\355"
#define ZSTR_GPSALT            "\007\001\364\354"
#define ZSTR_GPSDATETIME       "\004\377\354\373"
#define ZSTR_GPS               "\007\020\023"
#define ZSTR_BATT1_VOLTAGE     "\022\002\034\026"
#define ZSTR_BATT2_VOLTAGE     "\022\002\035\026"
#define ZSTR_BATT1_CURRENT     "\022\002\034\001"
#define ZSTR_BATT2_CURRENT     "\022\002\035\001"
#define ZSTR_BATT1_CONSUMPTION "\022\002\034\003"
#define ZSTR_BATT2_CONSUMPTION "\022\002\035\003"
#define ZSTR_BATT1_TEMP        "\022\002\034\024"
#define ZSTR_BATT2_TEMP        "\022\002\035\024"
#define ZSTR_RB_STATE          "\022\002\023"
#define ZSTR_CHANS_STATE       "\022\002\003\023"
#define ZSTR_RX_RSSI1          "\034\022\023\023"
#define ZSTR_RX_RSSI2          "\035\022\023\023"
#define ZSTR_RX_QUALITY        "\022\021\364\347"
#define ZSTR_RX_SNR            "\022\023\016\022"
#define ZSTR_RX_NOISE          "\022\016\355\373"
#define ZSTR_ANTENNA           "\001\016\024"
#define ZSTR_RF_MODE           "\022\006\015\004"
#define ZSTR_TX_POWER          "\024\020\027\022"
#define ZSTR_TX_RSSI           "\024\022\023\023"
#define ZSTR_TX_QUALITY        "\024\021\364\347"
#define ZSTR_RX_RSSI_PERC      "\022\022\023\020"
#define ZSTR_RX_RF_POWER       "\022\020\027\022"
#define ZSTR_TX_RSSI_PERC      "\024\022\023\020"
#define ZSTR_TX_RF_POWER       "\024\020\027\022"
#define ZSTR_TX_FPS            "\024\006\020\023"
#define ZSTR_TX_SNR            "\024\023\016\022"
#define ZSTR_TX_NOISE          "\024\016\355\373"
#define ZSTR_PITCH             "\020\354\375\370"
#define ZSTR_ROLL              "\022\361\364\364"
#define ZSTR_YAW               "\031\377\351"
#define ZSTR_FLIGHT_MODE       "\006\015"
#define ZSTR_THROTTLE          "\024\370\356"
#define ZSTR_QOS_A             "\006\374\373\001"
#define ZSTR_QOS_B             "\006\374\373\002"
#define ZSTR_QOS_L             "\006\374\373\014"
#define ZSTR_QOS_R             "\006\374\373\022"
#define ZSTR_QOS_F             "\006\014\355\355"
#define ZSTR_QOS_H             "\010\361\364\374"
#define ZSTR_BIND              "\002\011\016\004"
#define ZSTR_LAP_NUMBER        "\014\377\360 "
#define ZSTR_GATE_NUMBER       "\007\377\354\373"
#define ZSTR_LAP_TIME          "\014\377\360\024"
#define ZSTR_GATE_TIME         "\007\354\373\024"
#define ZSTR_ESC_VOLTAGE       "\005\355\375\026"
#define ZSTR_ESC_CURRENT       "\005\355\375\001"
#define ZSTR_ESC_RPM           "\005\356\360\363"
#define ZSTR_ESC_CONSUMPTION   "\005\355\375\003"
#define ZSTR_ESC_TEMP          "\005\355\375\024"
#define ZSTR_SD1_CHANNEL       "\003\370\377\362"
#define ZSTR_GASSUIT_TEMP1     "\007\024\360\034"
#define ZSTR_GASSUIT_TEMP2     "\007\024\360\035"
#define ZSTR_GASSUIT_RPM       "\007\022\020\015"
#define ZSTR_GASSUIT_FLOW      "\007\006\364\361"
#define ZSTR_GASSUIT_CONS      "\007\006\353\373"
#define ZSTR_GASSUIT_RES_VOL   "\007\022\026\364"
#define ZSTR_GASSUIT_RES_PERC  "\007\022\020\375"
#define ZSTR_GASSUIT_MAX_FLOW  "\007\015\006\364"
#define ZSTR_GASSUIT_AVG_FLOW  "\007\001\006\364"
#define ZSTR_SBEC_VOLTAGE      "\002\373\375\026"
#define ZSTR_SBEC_CURRENT      "\002\373\375\001"
#define ZSTR_RB3040_EXTRA_STATE        "\022\002\005\023"
#define ZSTR_RB3040_CHANNEL1           "\003\010\034\001"
#define ZSTR_RB3040_CHANNEL2           "\003\010\035\001"
#define ZSTR_RB3040_CHANNEL3           "\003\010\036\001"
#define ZSTR_RB3040_CHANNEL4           "\003\010\037\001"
#define ZSTR_RB3040_CHANNEL5           "\003\010\040\001"
#define ZSTR_RB3040_CHANNEL6           "\003\010\041\001"
#define ZSTR_RB3040_CHANNEL7           "\003\010\042\001"
#define ZSTR_RB3040_CHANNEL8           "\003\010\043\001"
#define ZSTR_ESC_VIN                   "\005\026\011\016"
#define ZSTR_ESC_TFET                  "\024\006\005\024"
#define ZSTR_ESC_CUR                   "\005\003\025\022"
#define ZSTR_ESC_TBEC                  "\024\002\005\003"
#define ZSTR_ESC_BCUR                  "\003\002\005\003"
#define ZSTR_ESC_VBEC                  "\026\002\005\003"
#define ZSTR_ESC_THR                   "\005\024\010\022"
#define ZSTR_ESC_POUT                  "\005\017\025\024"
#define ZSTR_SMART_BAT_BTMP            "\002\024\363\360"
#define ZSTR_SMART_BAT_BCUR            "\002\003\353\356"
#define ZSTR_SMART_BAT_BCAP            "\002\025\355\373"
#define ZSTR_SMART_BAT_MIN_CEL         "\003\014\015\367"
#define ZSTR_SMART_BAT_MAX_CEL         "\003\014\015\377"
#define ZSTR_SMART_BAT_CYCLES          "\003\347\375\364"
#define ZSTR_SMART_BAT_CAPACITY        "\002\003\360\024"
#define ZSTR_CL01                      "\003\373\364\034"
#define ZSTR_CL02                      "\003\373\364\035"
#define ZSTR_CL03                      "\003\373\364\036"
#define ZSTR_CL04                      "\003\373\364\037"
#define ZSTR_CL05                      "\003\373\364\040"
#define ZSTR_CL06                      "\003\373\364\041"
#define ZSTR_CL07                      "\003\373\364\042"
#define ZSTR_CL08                      "\003\373\364\043"
#define ZSTR_CL09                      "\003\373\364\044"
#define ZSTR_CL10                      "\003\364\034\033"
#define ZSTR_CL11                      "\003\364\034\034"
#define ZSTR_CL12                      "\003\364\034\035"
#define ZSTR_CL13                      "\003\364\034\036"
#define ZSTR_CL14                      "\003\364\034\037"
#define ZSTR_CL15                      "\003\364\034\040"
#define ZSTR_CL16                      "\003\364\034\041"
#define ZSTR_CL17                      "\003\364\034\042"
#define ZSTR_CL18                      "\003\364\034\043"
#define ZSTR_FRAME_RATE                "\006\022\377\354"
#define ZSTR_TOTAL_LATENCY             "\024\014\377\354"
#define ZSTR_VTX_FREQ                  "\026\006\356\357"
#define ZSTR_VTX_PWR                   "\026\020\351\356"
#define ZSTR_VTX_CHAN                  "\026\003\370\362"
#define ZSTR_VTX_BAND                  "\026\002\377\362"
#define ZSTR_SERVO_CURRENT             "\023\356\352\001"
#define ZSTR_SERVO_VOLTAGE             "\023\356\352\026"
#define ZSTR_SERVO_TEMPERATURE         "\023\356\352\024"
#define ZSTR_SERVO_STATUS              "\023\356\352\023"
#define ZSTR_LOSS                      "\014\361\355\355"
#define ZSTR_SPEED                     "\023\360\374 "
#define ZSTR_FLOW                      "\006\364\361\351"

//
// HoTT Telemetry sensor names by Hott device
//
// naming convention: Name of device in capital letters (1 or 2) followed by sensor name in lower case letters
//
// example: GPal = GPS altitude, GAal = GAM altitude, Valt = Vario altitude, GAt2 = GAM temperature sensor 2
//
// T  = transmitter
// R  = receiver
// V  = Vario
// G  = GPS
// ES = ESC
// EA = EAM
//
// ssi = signal strength indicator
// qly = quality
// bt  = battery
// evt = HoTT warnings
// btm = battery lowest voltage
// vpk = VPack
// al or alt = altitude
// vv  = vertical velocity (climb rate)
// hd  = heading
// sp  = speed
// PS  = GPS coordinates
// di  = direction
// ns  = number of satellites
// cp  = capacity
// u   = voltage (may be followed by numner if device offers more voltage sensors
// i   = current (may be followed by numner if device offers more current sensors
// tmp or t = temperature (single t may be followed by numner if device offers more temperature sensors
// rp or r  = temperature (single r may be followed by numner if device offers more temperature sensors
// fl = fuel
//  
// TX
#define ZSTR_HOTT_ID_TX_RSSI_DL     "\024\355\355\367"
#define ZSTR_HOTT_ID_TX_LQI_DL      "\024\357\364\347"
// RX
#define ZSTR_HOTT_ID_RX_RSSI_UL     "\022\355\355\367"
#define ZSTR_HOTT_ID_RX_LQI_UL      "\022\357\364\347"
#define ZSTR_HOTT_ID_RX_VLT         "\022\376\354"
#define ZSTR_HOTT_ID_RX_TMP         "\022\354\363\360" 
#define ZSTR_HOTT_ID_RX_BAT_MIN     "\022\376\354\363"
#define ZSTR_HOTT_ID_RX_VPCK        "\022\352\360\365"
#define ZSTR_HOTT_ID_RX_EVENT       "\022\373\352\354"
// Vario
#define ZSTR_HOTT_ID_VARIO_ALT      "\026\377\364\354"
#define ZSTR_HOTT_ID_VARIO_VV       "\026\352\352"
#define ZSTR_HOTT_ID_VARIO_HDG      "\026\370\374\371"
// GPS
#define ZSTR_HOTT_ID_GPS_HDG        "\007\020\370\374"
#define ZSTR_HOTT_ID_GPS_SPEED      "\007\020\355\360"
#define ZSTR_HOTT_ID_GPS_LL         "\007\020\023"
#define ZSTR_HOTT_ID_GPS_DST        "\007\020\374\367"
#define ZSTR_HOTT_ID_GPS_ALT        "\007\020\377\364" 
#define ZSTR_HOTT_ID_GPS_VV         "\007\020\352\352"
#define ZSTR_HOTT_ID_GPS_NSATS      "\007\020\362\355"
// ESC
#define ZSTR_HOTT_ID_ESC_VLT        "\005\023\353\034"
#define ZSTR_HOTT_ID_ESC_CAP        "\005\023\375\360"
#define ZSTR_HOTT_ID_ESC_TMP        "\005\023\354\034" 
#define ZSTR_HOTT_ID_ESC_CUR        "\005\023\367\034"
#define ZSTR_HOTT_ID_ESC_RPM        "\005\023\356\360"
#define ZSTR_HOTT_ID_ESC_BEC_VLT    "\005\023\353\035"
#define ZSTR_HOTT_ID_ESC_BEC_CUR    "\005\023\367\035" 
#define ZSTR_HOTT_ID_ESC_BEC_TMP    "\005\023\354\035"
#define ZSTR_HOTT_ID_ESC_MOT_TMP    "\005\023\354\036"
// GAM
#define ZSTR_HOTT_ID_GAM_CELS       "\007\001\375\364"
#define ZSTR_HOTT_ID_GAM_VLT1       "\007\001\353\034"
#define ZSTR_HOTT_ID_GAM_VLT2       "\007\001\353\035"
#define ZSTR_HOTT_ID_GAM_TMP1       "\007\001\354\034"
#define ZSTR_HOTT_ID_GAM_TMP2       "\007\001\354\035"
#define ZSTR_HOTT_ID_GAM_FUEL       "\007\001\372\364"
#define ZSTR_HOTT_ID_GAM_RPM1       "\007\001\356\034"
#define ZSTR_HOTT_ID_GAM_ALT        "\007\001\377\364"
#define ZSTR_HOTT_ID_GAM_VV         "\007\001\352\352"
#define ZSTR_HOTT_ID_GAM_CUR        "\007\001\367"
#define ZSTR_HOTT_ID_GAM_VLT3       "\007\001\353\036"
#define ZSTR_HOTT_ID_GAM_CAP        "\007\001\375\360"
#define ZSTR_HOTT_ID_GAM_SPEED      "\007\001\355\360"
#define ZSTR_HOTT_ID_GAM_RPM2       "\007\001\356\035"
// EAM
#define ZSTR_HOTT_ID_EAM_CELS_L     "\005\001\375\034"
#define ZSTR_HOTT_ID_EAM_CELS_H     "\005\001\375\035"
#define ZSTR_HOTT_ID_EAM_VLT1       "\005\001\353\034"
#define ZSTR_HOTT_ID_EAM_VLT2       "\005\001\353\035"
#define ZSTR_HOTT_ID_EAM_TMP1       "\005\001\354\034"
#define ZSTR_HOTT_ID_EAM_TMP2       "\005\001\354\035"
#define ZSTR_HOTT_ID_EAM_ALT        "\005\001\377\364"
#define ZSTR_HOTT_ID_EAM_CUR        "\005\001\367"
#define ZSTR_HOTT_ID_EAM_VLT3       "\005\001\353\036"
#define ZSTR_HOTT_ID_EAM_CAP        "\005\001\375\360"
#define ZSTR_HOTT_ID_EAM_VV         "\005\001\352\352"
#define ZSTR_HOTT_ID_EAM_RPM        "\005\001\356\360"
#define ZSTR_HOTT_ID_EAM_SPEED      "\005\001\355\360"

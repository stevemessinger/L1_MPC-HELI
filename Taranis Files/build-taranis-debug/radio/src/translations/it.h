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

// IT translations author: Romolo Manfredini <romolo.manfredini@gmail.com>
// IT integrations for OpenTX V2.3.x author: Marco Robustini <robustinimarco@gmail.com>

/*
 * !!!!! DO NOT EDIT it.h - EDIT it.h.txt INSTEAD !!!!!!!
 *
 * In order to make translations easier it.h.txt is parsed and national
 * characters are replaced by bitmap codes. The result is it.h.
 *
 * See translate.py in the util folder for the list of character codes
 *
 * !!!!! DO NOT EDIT it.h - EDIT it.h.txt INSTEAD !!!!!!!
 */

/*
 * Formatting octal codes available in TR_ strings:
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

#define LEN_VBEEPMODE          TR("\005", "\010")
#define TR_VBEEPMODE           TR("Silen""Avvis""Notst""Tutti","Silente ""Avvisi\0 ""No Tasti""Tutti\0  ")

#define LEN_VBLMODE            TR("\005", "\006")
#define TR_VBLMODE             TR("OFF\0 ""Tasti""Stks\0""Tutti""ON\0  ", "Spenta""Tasti\0""Sticks""Tutti\0""Accesa")

#define LEN_TRNMODE            "\003"
#define TR_TRNMODE             "OFF""+=\0"":="

#define LEN_TRNCHN             "\003"
#define TR_TRNCHN              "ch1ch2ch3ch4"

#define LEN_AUX_SERIAL_MODES   "\016"
#if defined(CLI) || defined(DEBUG)
#define TR_AUX_SERIAL_MODES    "Debug\0        ""Replica S-Port""Telemetria\0   ""SBUS Trainer\0 ""LUA\0          "
#else
#define TR_AUX_SERIAL_MODES    "OFF\0          ""Replica S-Port""Telemetria\0   ""SBUS Trainer\0 ""LUA\0          "
#endif

#define LEN_SWTYPES            "\006"
#define TR_SWTYPES             "Dis.\0 ""Toggle""2POS\0 ""3POS\0"

#define LEN_POTTYPES           TR("\013","\017")
#define TR_POTTYPES            TR("Dis.\0      ""Pot c. fer\0""Multipos\0  ""Pot\0       ", "Dis.\0          ""Pot. con centro""Inter. Multipos""Potenziometro\0 ")

#define LEN_SLIDERTYPES        "\006"
#define TR_SLIDERTYPES         "Disat.""Slider"

#define LEN_VLCD               "\006"
#define TR_VLCD                "NormalOptrex"

#define LEN_VPERSISTENT        "\015"
#define TR_VPERSISTENT         "NO\0          ""Volo\0        ""Reset Manuale"

#define LEN_COUNTRYCODES       TR("\002", "\007")
#define TR_COUNTRYCODES        TR("US""JP""EU", "America""Japan\0 ""Europa\0")

#if defined(RADIO_FAMILY_TBS)
#define LEN_USBMODES                   TR("\006", "\010")
#define TR_USBMODES                    TR("Ask\0  ""Joyst\0""Agent\0""SDCard""Serial", "Ask\0    ""Joystick""Agent\0 ""Storage\0""Serial\0 ")
#else
#define LEN_USBMODES           TR("\006", "\010")
#if defined(DEBUG)
#define TR_USBMODES            TR("Chied\0""Joyst\0""SDCard""Serial", "Chiedi\0 ""Joystick""Storage\0""Seriale\0 ")
#else
#define TR_USBMODES            TR("Chied\0""Joyst\0""SDCard""Telem\0", "Chiedi\0 ""Joystick""Storage\0""Telem\0  ")
#endif
#endif

#define LEN_JACKMODES                  "\007"
#define TR_JACKMODES                   "Chiedi\0   ""Audio\0 ""Trainer"

#define LEN_TELEMETRY_PROTOCOLS "\017"
#define TR_TELEMETRY_PROTOCOLS "FrSky S.PORT\0  ""FrSky D\0       ""FrSky D (cable)""TBS Crossfire\0 ""Spektrum\0      ""AFHDS2A IBUS\0  ""Multi Telemetry"

#define TR_MULTI_CUSTOM        "Person."

#define LEN_VTRIMINC           "\006"
#define TR_VTRIMINC            "Exp   ""ExFine""Fine  ""Medio ""Ampio "

#define LEN_VDISPLAYTRIMS      "\006"
#define TR_VDISPLAYTRIMS       "No\0   ""Cambio""Si\0 "

#define LEN_VBEEPCOUNTDOWN     "\006"
#define TR_VBEEPCOUNTDOWN      "NienteSuoni\0Voce\0 Vibra\0"

#define LEN_VVARIOCENTER       "\006"
#define TR_VVARIOCENTER        "Tono\0 ""Silenz."

#define LEN_CURVE_TYPES        "\010"
#define TR_CURVE_TYPES         "Fisso\0  ""Modific."

#define LEN_RETA123            "\001"

#if defined(PCBHORUS)
  #define TR_RETA123           "DEMA13245LR"
#elif defined(PCBX9E)
  #define TR_RETA123           "DEMA123SDSD"
#elif defined(PCBTARANIS) || defined(REVX)
  #define TR_RETA123           "DEMA123SD"
#elif defined(PCBSKY9X)
  #define TR_RETA123           "DEMA123a"
#else
  #define TR_RETA123           "DEMA123"
#endif

#if defined(PCBSKY9X) && defined(REVX)
  #define LEN_VOUTPUT_TYPE     "\011"
  #define TR_VOUTPUT_TYPE      "OpenDrain""PushPull\0"
#endif

#define LEN_VCURVEFUNC         "\003"
#define TR_VCURVEFUNC          "---""x>0""x<0""|x|""f>0""f<0""|f|"

#define LEN_VMLTPX             "\005"
#define TR_VMLTPX              "Add. ""Molt.""Sost."

#define LEN_VMLTPX2            "\002"
#define TR_VMLTPX2             "+=""*="":="

#define LEN_VMIXTRIMS          "\003"

#if defined(PCBHORUS)
  #define TR_VMIXTRIMS         "OFF""ON\0""Dir""Ele""Mot""Ale""T5\0""T6\0"
#else
  #define TR_VMIXTRIMS         "OFF""ON\0""Dir""Ele""Mot""Ale"
#endif

#if LCD_W >= 212
  #define TR_CSWTIMER          "Timer"
  #define TR_CSWSTICKY         "Stcky"
  #define TR_CSWRANGE          "Campo"
  #define TR_CSWSTAY           "Edge\0"
#else
  #define TR_CSWTIMER          "Tim\0 "
  #define TR_CSWSTICKY         "Glue\0"
    #define TR_CSWRANGE        "Rnge\0"
    #define TR_CSWSTAY         "Edge\0"
#endif

  #define TR_CSWEQUAL          "a=x\0 "

#define LEN_VCSWFUNC           "\005"
#define TR_VCSWFUNC            "---\0 " TR_CSWEQUAL "a\173x\0 ""a>x\0 ""a<x\0 " TR_CSWRANGE "|a|>x""|a|<x""AND\0 ""OR\0  ""XOR\0 " TR_CSWSTAY "a=b\0 ""a>b\0 ""a<b\0 ""\306}x\0 ""|\306|}x" TR_CSWTIMER TR_CSWSTICKY

#define LEN_TEXT_SIZE          "\010"
#define TR_TEXT_SIZE           "Standard""Tiny\0   ""Small\0  ""Mid\0    ""Double\0 "

#define LEN_VFSWFUNC           "\015"

#if defined(VARIO)
  #define TR_VVARIO            "Vario\0       "
#else
  #define TR_VVARIO            "[Vario]\0     "
#endif

#if defined(AUDIO)
  #define TR_SOUND             "Suona\0       "
#else
  #define TR_SOUND             "Beep\0        "
#endif

#if defined(HAPTIC)
  #define TR_HAPTIC            "Vibrazione\0  "
#else
  #define TR_HAPTIC            "[Vibrazione]\0"
#endif

#if defined(VOICE)
  #if defined(PCBSKY9X)
    #define TR_PLAY_TRACK      "Suona\0       "
  #else
    #define TR_PLAY_TRACK      "SuonaTraccia\0"
  #endif
  #define TR_PLAY_BOTH         "Play Both\0   "
  #define TR_PLAY_VALUE        TR("LeggiVal\0 ", "LeggiValore\0 ")
#else
  #define TR_PLAY_TRACK        "[Brano]\0     "
  #define TR_PLAY_BOTH         "[Play Both]\0 "
  #define TR_PLAY_VALUE        "[LeggiValore]"
#endif

#define TR_SF_BG_MUSIC        "Musica Sf\0   ""Musica Sf ||\0"

#if defined(SDCARD)
  #define TR_SDCLOGS           "Logs SDCard\0 "
#else
  #define TR_SDCLOGS           "[Logs SDCard]\0"
#endif

#if defined(GVARS)
  #define TR_ADJUST_GVAR       "Regola\0      "
#else
  #define TR_ADJUST_GVAR       "[RegolaVG]\0  "
#endif

#if defined(LUA)
  #define TR_SF_PLAY_SCRIPT   "Script Lua\0  "
#else
  #define TR_SF_PLAY_SCRIPT   "[Lua]\0       "
#endif

#if defined(DEBUG)
  #define TR_SF_TEST          "Test\0        "
#else
  #define TR_SF_TEST
#endif

#if defined(OVERRIDE_CHANNEL_FUNCTION) && LCD_W >= 212
  #define TR_SF_SAFETY        "Blocco\0      "
#elif defined(OVERRIDE_CHANNEL_FUNCTION)
  #define TR_SF_SAFETY        "Blocco\0      "
#else
  #define TR_SF_SAFETY        "---\0         "
#endif

#define TR_SF_SCREENSHOT       "Screenshot\0  "
#define TR_SF_RACING_MODE      "Racing Mode\0 "
#define TR_SF_RESERVE          "[riserva]   \0"

#define TR_VFSWFUNC            TR_SF_SAFETY "Maestro \0    ""Trim Instant.""Azzera\0      ""Set \0        " TR_ADJUST_GVAR "Volume\0      " "SetFailsafe\0 " "RangeCheck\0  " "ModuleBind\0  " TR_SOUND TR_PLAY_TRACK TR_PLAY_VALUE TR_SF_RESERVE TR_SF_PLAY_SCRIPT TR_SF_RESERVE TR_SF_BG_MUSIC TR_VVARIO TR_HAPTIC TR_SDCLOGS "Retroillum.\0 " TR_SF_SCREENSHOT TR_SF_RACING_MODE TR_SF_TEST

#define LEN_VFSWRESET          TR("\004", "\011")

#define TR_FSW_RESET_TELEM   TR("Telm", "Telemetr.")

#if LCD_W >= 212
  #define TR_FSW_RESET_TIMERS  "Timer 1\0 ""Timer 2\0 ""Timer 3\0 "
#else
  #define TR_FSW_RESET_TIMERS  "Tmr1""Tmr2""Tmr3"
#endif

#define TR_VFSWRESET           TR(TR_FSW_RESET_TIMERS "All\0" TR_FSW_RESET_TELEM, TR_FSW_RESET_TIMERS "Tutto\0    " TR_FSW_RESET_TELEM)

#define LEN_FUNCSOUNDS         TR("\004", "\006")
#define TR_FUNCSOUNDS          TR("Bp1\0""Bp2\0""Bp3\0""Wrn1""Wrn2""Chee""Rata""Tick""Sirn""Ring""SciF""Robt""Chrp""Tada""Crck""Alrm", "Beep1 ""Beep2 ""Beep3 ""Warn1 ""Warn2 ""Cheep ""Ratata""Tick  ""Siren ""Ring  ""SciFi ""Robot ""Chirp ""Tada  ""Crickt""AlmClk")

#define LEN_VTELEMCHNS         "\004"

  #define TR_TELEM_RESERVE     TR("[--]", "[---]")
  #define TR_TELEM_TIME        TR("Ora\0", "Ora\0 ")
  #define TR_RAS               TR("SWR\0", "SWR\0 ")
  #define TR_RX_BATT           TR("RxBt", "BatRx")
  #define TR_A3_A4             TR("A3\0 ""A4\0 ", "A3\0  ""A4\0  ")
  #define TR_A3_A4_MIN         TR("A3-\0""A4-\0", "A3-\0 ""A4-\0 ")

#define TR_ASPD_MAX            TR("ASp+", "ASpd+")

#if LCD_W >= 212
  #define TR_TELEM_RSSI_RX     "RSSI\0"
#else
  #define TR_TELEM_RSSI_RX     TR("Rx\0 ", "Rx\0  ")
#endif

  #define TR_TELEM_TIMERS      TR("Tmr1""Tmr2""Tmr3", "Tmr1\0""Tmr2\0""Tmr3\0")

#define LENGTH_UNIT_IMP        "ft\0"
#define SPEED_UNIT_IMP         "mph"
#define LENGTH_UNIT_METR       "m\0 "
#define SPEED_UNIT_METR        "kmh"

#define LEN_VUNITSSYSTEM     TR("\006", "\011")
#define TR_VUNITSSYSTEM      TR("Metric""Imper.", "Metriche\0""Imperiali")
#define LEN_VTELEMUNIT       "\003"
#define TR_VTELEMUNIT        "-\0 ""V\0 ""A\0 ""mA\0""kts""m/s""f/s""kmh""mph""m\0 ""ft\0""@C\0""@F\0""%\0 ""mAh""W\0 ""mW\0""dB\0""rpm""g\0 ""@\0 ""rad""ml\0""fOz""mlm""Hz\0""mS\0""uS\0""km\0""dbm"

#define STR_V                  (STR_VTELEMUNIT+1)
#define STR_A                  (STR_VTELEMUNIT+4)

#define LEN_VTELEMSCREENTYPE "\006"
#define TR_VTELEMSCREENTYPE  "Niente""Valori""Barre\0""Script"

#define LEN_GPSFORMAT          "\004"
#define TR_GPSFORMAT           "HMS NMEA"

#define LEN2_VTEMPLATES        12
#define LEN_VTEMPLATES         "\014"
#define TR_TEMPLATE_CLEAR_MIXES        "Canc. Mixer "
#define TR_TEMPLATE_SIMPLE_4CH         "Semplice 4CH"
#define TR_TEMPLATE_STICKY_TCUT        "Coda-V      "
#define TR_TEMPLATE_VTAIL              "V-Tail      "
#define TR_TEMPLATE_DELTA              "Elevon\\Delta"
#define TR_TEMPLATE_ECCPM              "eCCPM       "
#define TR_TEMPLATE_HELI               "Heli Setup  "
#define TR_TEMPLATE_SERVO_TEST         "Test Servo  "

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

#define TR_STICKS_VSRCRAW      "\307Dir""\307Ele""\307Mot""\307Ale"

#if defined(PCBHORUS)
  #define TR_TRIMS_VSRCRAW     "\313Dir""\313Ele""\313Mot""\313Ale""\313T5\0""\313T6\0"
#else
  #define TR_TRIMS_VSRCRAW     TR("TrmR""TrmE""TrmT""TrmA", "\313Dir""\313Ele""\313Mot""\313Ale")
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
#define TR_EXTRA_VSRCRAW     "Batt""Time""GPS\0" TR_RESERVE_VSRCRAW TR_RESERVE_VSRCRAW TR_RESERVE_VSRCRAW TR_RESERVE_VSRCRAW "Tmr1""Tmr2""Tmr3"

#define LEN_VTMRMODES          "\003"
#define TR_VTMRMODES           "OFF""ABS""MOs""MO%""MOt"

#define LEN_VTRAINERMODES              "\022"
#define TR_VTRAINER_MASTER_JACK        "Maestro/Jack\0     "
#define TR_VTRAINER_SLAVE_JACK         "Allievo/Jack\0     "
#define TR_VTRAINER_MASTER_SBUS_MODULE "Master/Modulo SBUS"
#define TR_VTRAINER_MASTER_CPPM_MODULE "Master/Modulo CPPM"
#define TR_VTRAINER_MASTER_BATTERY     "Master/Seriale\0    "
#define TR_VTRAINER_BLUETOOTH          TR("Master/BT\0        ""Slave/BT\0         ", "Master/Bluetooth\0 ""Slave/Bluetooth\0  ")
#define TR_VTRAINER_MULTI              "Master/Multi\0     "
#define TR_VTRAINER_SPORT_SBUS         "Master/SBUS-Sport\0"

#define LEN_VFAILSAFE          "\013"
#define TR_VFAILSAFE           "Non settato""Mantieni\0  ""Personali\0 ""No impulsi\0""Ricevente\0 "


#define LEN_VSENSORTYPES        "\012"
#define TR_VSENSORTYPES        "Custom\0   ""Calcolato\0"

#define LEN_VFORMULAS          "\011"
#define TR_VFORMULAS           "Somma\0   ""Media\0   ""Min\0     ""Max\0     ""Moltipl\0 ""Totalizza""Cella\0  ""Consumo\0 ""Distanza\0"

#define LEN_VPREC              "\004"
#define TR_VPREC               "0.--""0.0 ""0.00"

#define LEN_VCELLINDEX         "\010"
#define TR_VCELLINDEX          "Minore\0 ""1\0      ""2\0      ""3\0      ""4\0      ""5\0      ""6\0      ""Maggiore""Delta\0"

#define LEN_GYROS                      "\004"
#define TR_GYROS                       "GyrX""GyrY"

// ZERO TERMINATED STRINGS
#if defined(COLORLCD)
  #define INDENT               "\007"
  #define LEN_INDENT           1
  #define INDENT_WIDTH         12
  #define BREAKSPACE           "\036"
#else
  #define INDENT               "\001"
  #define LEN_INDENT           1
  #define INDENT_WIDTH         (FW/2)
  #define BREAKSPACE           " "
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

#define TR_MENUWHENDONE        CENTER "\007" TR_ENTER " Conferma"
#define TR_FREE                "Disp."
#define TR_DELETEMODEL         "CANCELLA" BREAKSPACE "MODELLO"
#define TR_COPYINGMODEL        "Copia in corso.."
#define TR_MOVINGMODEL         "Spostamento..."
#define TR_LOADINGMODEL        "Caricamento..."
#define TR_NAME                "Nome"
#define TR_MODELNAME            TR("Nome Mode.", "Nome Modello")
#define TR_PHASENAME           "Nome fase"
#define TR_MIXNAME             "Nome mix"
#define TR_INPUTNAME           "Nome Ingr."
  #define TR_EXPONAME          "Nome expo"
#define TR_BITMAP              "Immagine"
#define TR_TIMER               TR("Timer", "Timer ")
#define TR_ELIMITS             TR("Limiti.E", "Limiti Estesi")
#define TR_ETRIMS              TR("Trim Ext", "Trim Estesi")
#define TR_TRIMINC             "Passo Trim"
#define TR_DISPLAY_TRIMS       TR("Mos. Trims", "Mostra Trims")
#define TR_TTRACE              TR("So. motore", INDENT "Sorgente Motore")
#define TR_TTRIM               TR("Trim Mot.", INDENT "Trim Motore")
#define TR_TTRIM_SW            TR("T-Trim-Sw", INDENT "Trim switch")
#define TR_BEEPCTR             TR("Beep al c.", "Beep al centro")
#define TR_USE_GLOBAL_FUNCS    TR("Funz. Glob.", "Usa Funz. Globali")
#if defined(PCBSKY9X) && defined(REVX)
  #define TR_OUTPUT_TYPE       INDENT "Uscita"
#endif
#define TR_PROTOCOL            TR("Protoc.", "Protocollo")
#define TR_PPMFRAME            INDENT "Frame PPM"
#define TR_REFRESHRATE         TR(INDENT "Refresh", INDENT "Refresh rate")
#define STR_WARN_BATTVOLTAGE   TR(INDENT "Uscita VBAT: ", INDENT "Att.: livel. uscita VBAT: ")
#define TR_WARN_5VOLTS         "Attenzione: il livello d'uscita è 5 Volts"
#define TR_MS                  "ms"
#define TR_FREQUENCY           INDENT "Frequenza"
#define TR_SWITCH              "Inter."
#define TR_TRIMS               "Trims"
#define TR_FADEIN              "Diss.In"
#define TR_FADEOUT             "Diss.Out"
#define TR_DEFAULT             "(Predefinita)"
#define TR_CHECKTRIMS          CENTER "\006Contr.\012Trims"
#define OFS_CHECKTRIMS         CENTER_OFS+(9*FW)
#define TR_SWASHTYPE           "Tipo Ciclico"
#define TR_COLLECTIVE          TR("Collettivo", "Origine Collettivo")
#define TR_AILERON             TR("Cic. later.", "Sorg. cic. later.")
#define TR_ELEVATOR            TR("Cic. long.", "Sorg. cic. long.")
#define TR_SWASHRING           "Anello Ciclico"
#define TR_ELEDIRECTION        TR("Direzione ELE", "Direzione cic. long.")
#define TR_AILDIRECTION        TR("Direzione AIL", "Direzione cic. lat.")
#define TR_COLDIRECTION        TR("Direzione PIT", "Direzione passo coll.")
#define TR_MODE                "Modo"
#define TR_SUBTYPE             INDENT "Sottotipo"
#define TR_NOFREEEXPO          "Expo pieni!"
#define TR_NOFREEMIXER         "Mixer pieni!"
#define TR_SOURCE              "Sorg."
#define TR_WEIGHT              "Peso"
#define TR_EXPO                "Espo"
#define TR_SIDE                "Lato"
#define TR_DIFFERENTIAL        "Differ"
#define TR_OFFSET              "Offset"
#define TR_TRIM                "Trim"
#define TR_DREX                "DRex"
#define DREX_CHBOX_OFFSET      30
#define TR_CURVE               "Curva"
#define TR_FLMODE              TR("Fase", "Fasi")
#define TR_MIXWARNING          "Avviso"
#define TR_OFF                 "OFF"
#define TR_ANTENNA             "Antenna"
#define TR_NO_INFORMATION      TR("No info", "No informazione")
#define TR_MULTPX              "MultPx"
#define TR_DELAYDOWN           "Post.Gi\201 "
#define TR_DELAYUP             "Post.Su"
#define TR_SLOWDOWN            "Rall.Gi\201 "
#define TR_SLOWUP              "Rall.Su"
#define TR_MIXES               "MIXER"
#define TR_CV                  "CV"
#define TR_GV                  TR("G", "GV")
#define TR_ACHANNEL            "A\002ingresso"
#define TR_RANGE               TR(INDENT "Inter.", INDENT "Intervallo")
#define TR_CENTER              INDENT "Centro"
#define TR_BAR                 "Barra"
#define TR_ALARM               TR(INDENT "Allar.", INDENT "Allarme")
#define TR_USRDATA             "Dati"
#define TR_BLADES              INDENT "Pale"
#define TR_SCREEN              "Schermo\001"
#define TR_SOUND_LABEL         "Suono"
#define TR_LENGTH              INDENT "Durata"
#define TR_BEEP_LENGTH         INDENT "Lung. Beep"
#define TR_SPKRPITCH           INDENT "Tono"
#define TR_HAPTIC_LABEL        "Vibrazione"
#define TR_HAPTICSTRENGTH      INDENT "Forza"
#define TR_GYRO_LABEL                  "Gyro"
#define TR_GYRO_OFFSET                 "Offset"
#define TR_GYRO_MAX                    "Max"
#define TR_CONTRAST            "Contrasto"
#define TR_ALARMS_LABEL        "Allarmi"
#define TR_BATTERY_RANGE       TR("Int. Batt.", "Intervallo batteria")
#define TR_BATTERYWARNING      TR(INDENT "Avv. Batt.", INDENT "Avviso Batteria")
#define TR_INACTIVITYALARM     INDENT "Inattivit\200"
#define TR_MEMORYWARNING       TR(INDENT "Avv. Mem.", INDENT "Avviso Memoria")
#define TR_ALARMWARNING        INDENT "Spegni suono"
#define TR_RSSISHUTDOWNALARM   TR(INDENT "RSSI Shut.", INDENT "Control. RSSI al Shutdown")
#define TR_MODEL_STILL_POWERED "Ricevente ancora connessa"
#define TR_MODEL_SHUTDOWN              "Shutdown?"
#define TR_PRESS_ENTER_TO_CONFIRM      "Premi [ENT] per confermare"
#define TR_THROTTLE_LABEL      "Motore"
#define TR_THROTTLEREVERSE     TR("Mot inv.", "Motore Inverso")
#define TR_MINUTEBEEP          "Minuto"
#define TR_BEEPCOUNTDOWN       TR(INDENT "Conto rov", INDENT "Conto rovescia")
#define TR_PERSISTENT          TR(INDENT "Persist.", INDENT "Persistente")
#define TR_BACKLIGHT_LABEL     TR("Retroillu.", "Retroilluminazione")
#define TR_GHOST_MENU_LABEL            "GHOST MENU"
#define TR_STATUS                      "Status"
#define TR_RAW_12BITS                  "Raw 12 bits"
#define TR_BLDELAY             INDENT "Durata"
#define TR_BLONBRIGHTNESS      TR(INDENT "Lumin. ON", INDENT "Luminosit\200 ON")
#define TR_BLOFFBRIGHTNESS     TR(INDENT "Lumin. OFF", INDENT "Luminosit\200 OFF")
#define TR_KEYS_BACKLIGHT              "Keys backlight"
#define TR_BLCOLOR             INDENT "Colore"
#define TR_SPLASHSCREEN        TR("Sch. avvio", "Schermata d'avvio")
#define TR_PWR_ON_DELAY        "Rit. accens."
#define TR_PWR_OFF_DELAY       "Rit. spegni."
#define TR_THROTTLEWARNING     TR(INDENT "All. Mot.", INDENT "Allarme Motore")
#define TR_SWITCHWARNING       TR(INDENT "Avv. Int.", INDENT "Avviso Interr.")
#define TR_POTWARNINGSTATE     TR(INDENT "Pot&Slid.", INDENT "Pots & sliders")
#define TR_SLIDERWARNING       TR(INDENT "Pos. slid.", INDENT "Posizione slider")
#define TR_POTWARNING          TR(INDENT "Avv. Pot.", INDENT "Avviso Pot.")
#define TR_TIMEZONE            "Ora locale"
#define TR_ADJUST_RTC          TR(INDENT "Agg. RTC", INDENT "Aggiusta RTC")
#define TR_GPS                 "GPS"
#define TR_RXCHANNELORD        "Ordine CH RX"
#define TR_STICKS              "Sticks"
#define TR_POTS                "Pot."
#define TR_SWITCHES            "Interutt."
#define TR_SWITCHES_DELAY      TR("Rit. V. FV", "Ritardo Voce FV")
#define TR_SLAVE               "Allievo"
#define TR_MODESRC             "Modo\006% Origine"
#define TR_MULTIPLIER          "Moltiplica"
#define TR_CAL                 "Cal"
#define TR_VTRIM               "Trim- +"
#define TR_BG                  "BG:"
#if defined(PCBHORUS)
  #define TR_MENUTOSTART       "Premi [ENT] per partire"
  #define TR_SETMIDPOINT       "Centra sticks/pots/sliders e premi [ENT]"
  #define TR_MOVESTICKSPOTS    "Muovi sticks/pots/sliders e premi [ENT]"
#elif defined(COLORLCD)
  #define TR_MENUTOSTART       TR_ENTER " PER PART."
  #define TR_SETMIDPOINT       "CENTRA STICKS/SLIDERS"
  #define TR_MOVESTICKSPOTS    "MUOVI STICKS/POTS"
#else
  #define TR_MENUTOSTART       CENTER"\011" TR_ENTER " per Cal."
  #define TR_SETMIDPOINT       CENTER"\012SETTA CENTRO"
  #define TR_MOVESTICKSPOTS    CENTER"\010MUOVI STICK/POT"
#endif
#define TR_RXBATT              "Batt Rx:"
#define TR_TXnRX               "Tx:\0Rx:"
#define OFS_RX                 4
#define TR_ACCEL               "Acc:"
#define TR_NODATA              CENTER"DATI ASSENTI"
#define TR_US                         "us"
#define TR_TMIXMAXMS         "Tmix max"
#define TR_FREE_STACK     "Free stack"
#define TR_MENUTORESET         TR_ENTER" per azzerare"
#define TR_PPM_TRAINER         "TR"
#define TR_CH                  "CH"
#define TR_MODEL               "MODELLO"
#define TR_FM                  "FV"
#define TR_MIX                 "MIX"
#define TR_EEPROMLOWMEM        "EEPROM quasi piena!"
#define TR_ALERT               "\016ALERT"
#define TR_PRESSANYKEYTOSKIP   "\010Premi un tasto"
#define TR_THROTTLENOTIDLE     TR("Motore non in posiz.", "Motore non in posizione")
#define TR_ALARMSDISABLED      "Avvisi Disattivati!"
#define TR_PRESSANYKEY         "\010Premi un tasto"
#define TR_BADEEPROMDATA       "Dati corrotti!"
#define TR_BAD_RADIO_DATA      "Dati radio errati"
#define TR_EEPROMFORMATTING    "Formatto EEPROM..."
#define TR_STORAGE_FORMAT      "Preparazione storage"
#define TR_EEPROMOVERFLOW      "EEPROM Piena"
#define TR_MENURADIOSETUP      "CONFIGURA TX"
#define TR_MENUDATEANDTIME     "DATA ED ORA"
#define TR_MENUTRAINER         "MAESTRO/ALLIEVO"
#define TR_MENUSPECIALFUNCS    "FUNZIONI GLOBALI"
#define TR_MENUVERSION         "VERSIONE"
#define TR_MENU_RADIO_SWITCHES            "DIAG"
#define TR_MENU_RADIO_ANALOGS             "ANAS"
#define TR_MENUCALIBRATION     "CALIBRAZIONE"
#if defined(COLORLCD)
  #define TR_TRIMS2OFFSETS     "Trims => Subtrims"
#else
  #define TR_TRIMS2OFFSETS     "\006Trims  => Offset "
#endif
#define TR_CHANNELS2FAILSAFE  "Canali => Failsafe"
#define TR_CHANNEL2FAILSAFE   "Canale => Failsafe"
#define TR_MENUMODELSEL        "MODELLI"
#define TR_MENUSETUP           "CONFIGURA"
#define TR_MENUFLIGHTMODE     "FASE DI VOLO"
#define TR_MENUFLIGHTMODES    "FASI DI VOLO"
#define TR_MENUHELISETUP       "CONFIGURA ELI"

  #define TR_MENUINPUTS        "INGRESSI"
  #define TR_MENULIMITS        "USCITE"

#define TR_MENUCURVES          "CURVE"
#define TR_MENUCURVE           "CURVA"
#define TR_MENULOGICALSWITCH   "INTER. PERSON."
#define TR_MENULOGICALSWITCHES TR("INTER. LOGICI", "INTERRUTTORI LOGICI")
#define TR_MENUCUSTOMFUNC      TR("FUNZ. SPECIALI", "FUNZIONI SPECIALI")
#define TR_MENUCUSTOMSCRIPTS   "SCRIPTS UTENTE"
#define TR_MENUTELEMETRY       "TELEMETRIA"
#define TR_MENUTEMPLATES       "ESEMPI GUIDA"
#define TR_MENUSTAT            "STATO"
#define TR_MENUDEBUG           "DEBUG"
#define TR_MONITOR_CHANNELS1   "MONITOR CANALI 1/8"
#define TR_MONITOR_CHANNELS2   "MONITOR CANALI 9/16"
#define TR_MONITOR_SWITCHES    "MONITOR INTERRUTTORI LOGICI"
#define TR_MONITOR_CHANNELS3   "MONITOR CANALI 17/24"
#define TR_MONITOR_CHANNELS4   "MONITOR CANALI 25/32"
#define TR_MONITOR_OUTPUT_DESC "Uscite"
#define TR_MONITOR_MIXER_DESC  "Mixers"
#define TR_RECEIVER_NUM        TR("RxNum", "Ricevente N.")
#define TR_RECEIVER            "Ricevente"
#define TR_MULTI_RFTUNE        TR("Tune RF", "Tune fine Freq. RF")
#define TR_MULTI_RFPOWER               "RF power"
#define TR_MULTI_WBUS                  "Output"
#define TR_MULTI_TELEMETRY     "Telemetria"
#define TR_MULTI_VIDFREQ       TR("Freq. video", "Frequenza video")
#define TR_RFPOWER             "Potenza RF"
#define TR_MULTI_FIXEDID               TR(INDENT"ID fisso", INDENT"ID Fisso")
#define TR_MULTI_OPTION        TR(INDENT"Opzione", INDENT"Opzione valore")
#define TR_MULTI_AUTOBIND      TR(INDENT "Ass. Ch.",INDENT "Associa al canale")
#define TR_DISABLE_CH_MAP              TR("No Ch. map", "Disab. mappa Ch.")
#define TR_DISABLE_TELEM               TR("No Telem", "Telem. disabil.")
#define TR_MULTI_DSM_AUTODTECT TR(INDENT "Trova autom.", INDENT "Autoril. il formato")
#define TR_MULTI_LOWPOWER      TR(INDENT "Bassa pot.", INDENT "Modo bassa potenza")
#define TR_MULTI_LNA_DISABLE            INDENT "LNA disable"
#define TR_MODULE_TELEMETRY            TR(INDENT "S.Port", INDENT "Link S.Port")
#define TR_MODULE_TELEM_ON             TR("ON", "Abilitato")
#define TR_DISABLE_INTERNAL         TR("Disatt. RF int.", "Disattiva RF interna")
#define TR_MODULE_NO_SERIAL_MODE       TR("!modo seriale", "Non in modo seriale")
#define TR_MODULE_NO_INPUT             TR("Nessun input", "Nessun input seriale")
#define TR_MODULE_NO_TELEMETRY         TR3("No telemetria", "MULTI_TELEMETRY disattivato", "Nessuna telem. (abilita MULTI_TELEMETRY)")
#define TR_MODULE_WAITFORBIND          "Associa per caricare il protocollo"
#define TR_MODULE_BINDING              "Associa"
#define TR_MODULE_UPGRADE_ALERT        TR3("Richiede agg.", "Richiede agg. modulo", "Modulo\036Richiede agg.")
#define TR_MODULE_UPGRADE              TR("Cons. agg.", "Consiglio agg. modulo")
#define TR_PULSE_RATE                  "Pulse rate"
#define TR_LINK_SPEED                  "Link speed"
#define TR_REBIND                      "Richiede associazione"
#define TR_REG_OK                      "Registrazione ok"
#define TR_BIND_OK                     "Associazione riuscita"
#define TR_BINDING_CH1_8_TELEM_ON               "Ch1-8 Telem ON"
#define TR_BINDING_CH1_8_TELEM_OFF               "Ch1-8 Telem OFF"
#define TR_BINDING_CH9_16_TELEM_ON               "Ch9-16 Telem ON"
#define TR_BINDING_CH9_16_TELEM_OFF               "Ch9-16 Telem OFF"
#define TR_PROTOCOL_INVALID            TR("Prot. invalido", "Protocollo invalido")
#define TR_MODULE_STATUS                TR(INDENT "Stato", INDENT "Stato del modulo")
#define TR_MODULE_SYNC                 TR(INDENT "Sinc.", INDENT "Sinc. del modulo")
#define TR_MULTI_SERVOFREQ     TR("Servo rate", "Aggiorna servo rate")
#define TR_MULTI_MAX_THROW             TR("Escurs. mass.", "Abilita escurs. mass.")
#define TR_MULTI_RFCHAN                TR("Canale RF", "Selez. canale RF")
#define TR_SYNCMENU            "[Sync]"
#define TR_LIMIT               INDENT "Limiti"
#define TR_MINRSSI             "Min. RSSI"
#define TR_LATITUDE            "Latitud."
#define TR_LONGITUDE           "Longitu."
#define TR_GPSCOORD            TR("GPS Coords", INDENT "Formato Coordinate")
#define TR_VARIO               TR("Vario", "Variometro")
#define TR_PITCH_AT_ZERO       INDENT "Tono a Zero"
#define TR_PITCH_AT_MAX        INDENT "Tono al Max"
#define TR_REPEAT_AT_ZERO      INDENT "Ripeti a Zero"
#define TR_SHUTDOWN            "ARRESTO.."
#define TR_SAVEMODEL           "Salvataggio dati modello"
#define TR_BATT_CALIB          TR("Calibra batt.", "Calibr. batteria")
#define TR_CURRENT_CALIB       "Calibra corr."
#define TR_VOLTAGE             TR(INDENT "Voltagg.",INDENT "Voltaggio")
#define TR_CURRENT             TR(INDENT "Corrente",INDENT "Corrente")
#define TR_SELECT_MODEL        TR("Scegli mod.", "Scegli Modello")
#define TR_SELECT_MODE                 "Select mode"
#define TR_CREATE_CATEGORY     "Crea Categoria"
#define TR_RENAME_CATEGORY     "Rinomina Categoria"
#define TR_DELETE_CATEGORY     "Cancella Categoria"
#define TR_CREATE_MODEL        "Crea Modello"
#define TR_DUPLICATE_MODEL     "Duplica Modello"
#define TR_COPY_MODEL          "Copia Modello"
#define TR_MOVE_MODEL          "Sposta Modello"
#define TR_BACKUP_MODEL        "Salva Modello"
#define TR_DELETE_MODEL        TR("Elim. Modello", "Elimina Modello")
#define TR_RESTORE_MODEL       TR("Ripr. Modello", "Ripristina Modello")
#define TR_DELETE_ERROR        "Errore cancell."
#define TR_CAT_NOT_EMPTY       "Categoria non vuota"
#define TR_SDCARD_ERROR        TR("Errore SD", "Errore SDCard")
#define TR_NO_SDCARD           "No SDCard"
#define TR_WAITING_FOR_RX              "Attendo la RX..."
#define TR_WAITING_FOR_TX              "Attendo la TX..."
#define TR_WAITING_FOR_MODULE          TR("Waiting module", "Waiting for module...")
#define TR_NO_TOOLS                    "Tools non disp."
#define TR_NORMAL                      "Normale"
#define TR_NOT_INVERTED                "No inv."
#define TR_NOT_CONNECTED               "!Connesso"
#define TR_CONNECTED                   "Connesso"
#define TR_FLEX_915                    "Flex 915MHz"
#define TR_FLEX_868                    "Flex 868MHz"
#define TR_16CH_WITHOUT_TELEMETRY      TR("16CH senza telem.", "16CH senza telemetria")
#define TR_16CH_WITH_TELEMETRY         TR("16CH con telem.", "16CH con telemetria")
#define TR_8CH_WITH_TELEMETRY          TR("8CH con telem.", "8CH con telemetria")
#define TR_EXT_ANTENNA                 "Antenna Ext."
#define TR_PIN                         "Pin"
#define TR_UPDATE_RX_OPTIONS           "Agg. opzioni RX?"
#define TR_UPDATE_TX_OPTIONS           "Agg. opzioni TX?"
#define TR_MODULES_RX_VERSION          "Versioni Moduli / RX"
#define TR_MENU_MODULES_RX_VERSION     "VERSIONI MODULI / RX"
#define TR_MENU_FIRM_OPTIONS           "OPZIONI FIRMWARE"
#define TR_GYRO                        "Gyro"
#define TR_STICKS_POTS_SLIDERS         "Interruttori/Pots/Sliders"
#define TR_PWM_STICKS_POTS_SLIDERS     "PWM Interruttori/Pots/Sliders"
#define TR_RF_PROTOCOL                 "Protocollo RF"
#define TR_MODULE_OPTIONS              "Optioni modulo"
#define TR_POWER                       "Potenza"
#define TR_NO_TX_OPTIONS               "No Opzioni TX"
#define TR_RTC_BATT                    "RTC Batt"
#define TR_POWER_METER_EXT             "Power Meter (EST)"
#define TR_POWER_METER_INT             "Power Meter (INT)"
#define TR_SPECTRUM_ANALYSER_EXT       "Spectrum (EST)"
#define TR_SPECTRUM_ANALYSER_INT       "Spectrum (INT)"
#define TR_SDCARD_FULL                 "SDCard Piena"
#define TR_NEEDS_FILE                  "RICHIEDE FILE"
#define TR_EXT_MULTI_SPEC              "opentx-inv"
#define TR_INT_MULTI_SPEC              "stm-opentx-noinv"
#define TR_INCOMPATIBLE        "Incompatibile"
#define TR_WARNING             "AVVISO"
#define TR_EEPROMWARN          "EEPROM"
#define TR_STORAGE_WARNING     "STORAGE"
#define TR_EEPROM_CONVERTING   "Conversione EEPROM"
#define TR_THROTTLEWARN        "MOTORE"
#define TR_ALARMSWARN          "ALLARMI"
#define TR_SWITCHWARN          "CONTROLLI"
#define TR_FAILSAFEWARN        "FAILSAFE"
#define TR_TEST_WARNING        TR("PER TEST", "BUILD PER TEST")
#define TR_TEST_NOTSAFE        "Usare solo per test"
#define TR_WRONG_SDCARDVERSION TR("Richiede ver: ", "Richiede versione: ")
#define TR_WARN_RTC_BATTERY_LOW        "Batteria RTC scarica"
#define TR_WARN_MULTI_LOWPOWER         "Modalit\200 bassa pot."
#define TR_BATTERY                     "BATTERIA"
#define TR_WRONG_PCBREV        "PCB sbagliato"
#define TR_EMERGENCY_MODE      "MODALITA' EMERGENZA"
#define TR_PCBREV_ERROR        "Errore PCB"
#define TR_NO_FAILSAFE         "Failsafe" BREAKSPACE "non settato"
#define TR_KEYSTUCK            "Tasto bloccato"
#define TR_INVERT_THR          "Inverti Mot?"
#define TR_SPEAKER_VOLUME      INDENT "Volume Audio"
#define TR_LCD                 "LCD"
#define TR_BRIGHTNESS          INDENT "Luminosit\200"
#define TR_CPU_TEMP            "Temp CPU \016>"
#define TR_CPU_CURRENT         "Corrente\022>"
#define TR_CPU_MAH             "Consumo"
#define TR_COPROC              "CoProc."
#define TR_COPROC_TEMP         "Temp. MB \016>"
#define TR_CAPAWARNING         INDENT "Capacit\200 Bassa"
#define TR_TEMPWARNING         INDENT "Temp. Alta"
#define TR_FUNC                "Funz"
#define TR_V1                  "V1"
#define TR_V2                  "V2"
#define TR_DURATION            "Durata"
#define TR_DELAY               "Ritardo"
#define TR_SD_CARD             "SD Card"
#define TR_SDHC_CARD           "SD-HC Card"
#define TR_NO_SOUNDS_ON_SD     "No Suoni" BREAKSPACE "su SD"
#define TR_NO_MODELS_ON_SD     "No Model." BREAKSPACE "su SD"
#define TR_NO_BITMAPS_ON_SD    "No Immag." BREAKSPACE "su SD"
#define TR_NO_SCRIPTS_ON_SD    "No Scripts" BREAKSPACE "su SD"
#define TR_SCRIPT_SYNTAX_ERROR TR("Errore sintassi", "Script errore sintassi")
#define TR_SCRIPT_PANIC        "Script panic"
#define TR_SCRIPT_KILLED       "Script fermato"
#define TR_SCRIPT_ERROR        "Errore sconosciuto"
#define TR_PLAY_FILE           "Suona"
#define TR_DELETE_FILE         "Elimina"
#define TR_COPY_FILE           "Copia"
#define TR_RENAME_FILE         "Rinomina"
#define TR_ASSIGN_BITMAP       "Assegna immagine"
#define TR_ASSIGN_SPLASH       "Splash screen"
#define TR_EXECUTE_FILE        "Esegui"
#define TR_REMOVED             " rimosso"
#define TR_SD_INFO             "Informazioni"
#define TR_SD_FORMAT           "Formatta"
#define TR_NA                  "N/A"
#define TR_HARDWARE            "HARDWARE"
#define TR_FORMATTING          "Formattazione"
#define TR_TEMP_CALIB          "Temp. Calib."
#define TR_TIME                "Ora"
#define TR_MAXBAUDRATE         "Max Bauds"

#define TR_BLUETOOTH            "Bluetooth"
#define TR_BLUETOOTH_DISC       "Cerca"
#define TR_BLUETOOTH_INIT       "Iniz."
#define TR_BLUETOOTH_DIST_ADDR  "Ind. Dist."
#define TR_BLUETOOTH_LOCAL_ADDR "Ind. Loc."
#define TR_BLUETOOTH_PIN_CODE   "Codice PIN"
#define TR_BAUDRATE             "Baudrate BT"
#define LEN_BLUETOOTH_MODES     "\011"
#if defined(PCBX9E)
#define TR_BLUETOOTH_MODES      "---\0     ""Attivo\0 "
#else
#define TR_BLUETOOTH_MODES      "---\0     ""Telemetr.""Trainer\0"
#endif
#define TR_SD_INFO_TITLE       "INFO SD"
#define TR_SD_TYPE             "Tipo:"
#define TR_SD_SPEED            "Veloc.:"
#define TR_SD_SECTORS          "Settori:"
#define TR_SD_SIZE             "Dimens:"
#define TR_TYPE                INDENT "Tipo"
#define TR_GLOBAL_VARS         "Variabili Globali"
#define TR_GVARS               "V.GLOBALI"
#define TR_GLOBAL_VAR          "Variabile globale"
#define TR_MENUGLOBALVARS      "VARIABILI GLOBALI"
#define TR_OWN                 "Fase"
#define TR_DATE                "Data"
#define TR_MONTHS              { "Gen", "Feb", "Mar", "Apr", "Mag", "Giu", "Lug", "Ago", "Set", "Ott", "Nov", "Dic" }
#define TR_ROTARY_ENCODER      "R.E."
#define TR_INVERT_ROTARY       "Invert Rotary"
#define TR_CHANNELS_MONITOR    "MONITOR CANALI"
#define TR_MIXERS_MONITOR      "MONITOR MIXER"
#define TR_PATH_TOO_LONG       "Path troppo lungo"
#define TR_VIEW_TEXT           "Vedi testo"
#define TR_FLASH_BOOTLOADER            TR("Prog. bootloader", "Programma bootloader")
#define TR_FLASH_EXTERNAL_DEVICE       TR("Prog. disp. est.", "Programma dispositivo esterno")
#define TR_FLASH_RECEIVER_OTA          TR("Prog. RX OTA", INDENT "Programma RX OTA")
#define TR_FLASH_RECEIVER_BY_EXTERNAL_MODULE_OTA          TR("Prog. RX da OTA est.", INDENT "Prog. RX da OTA esterno")
#define TR_FLASH_RECEIVER_BY_INTERNAL_MODULE_OTA          TR("Prog. RX da OTA int.", INDENT "Prog. RX da OTA interno")
#define TR_FLASH_FLIGHT_CONTROLLER_BY_EXTERNAL_MODULE_OTA TR("Prog. FC da OTA est.", INDENT "Prog. FC da OTA esterno")
#define TR_FLASH_FLIGHT_CONTROLLER_BY_INTERNAL_MODULE_OTA TR("Prog. FC da OTA int.", INDENT "Prog. FC da OTA interno")
#define TR_FLASH_BLUETOOTH_MODULE      TR("Prog. mod. BT", INDENT "Prog. modulo Bluetooth")
#define TR_FLASH_POWER_MANAGEMENT_UNIT TR("Prog. PMU", "Programma PMU")
#define TR_CURRENT_VERSION             TR("Vers. currente ", "Versione corrente: ")
#define TR_HW_REV                      "HW Rev"
#define TR_FLASH_EXTERNAL_ELRS         TR("Flash Ext. ELRS", "Flash External ELRS")
#define TR_FLASH_INTERNAL_MODULE       TR("Prog. modulo int.", "Programma modulo interno")
#define TR_FLASH_EXTERNAL_MODULE       TR("Prog. modulo est.", "Programma modulo esterno")
#define TR_FLASH_INTERNAL_MULTI        TR("Prog. MULTI int.", "Programma MULTI interno")
#define TR_FLASH_EXTERNAL_MULTI        TR("Prog. MULTI est.", "Programma MULTI esterno")
#define TR_FIRMWARE_UPDATE_ERROR       TR(INDENT "Errore agg. FW",INDENT "Errore aggiornamento firmware")
#define TR_FIRMWARE_UPDATE_SUCCESS     "Aggiornamento" BREAKSPACE "riuscito"
#define TR_WRITING                     "Scrittura..."
#define TR_CONFIRM_FORMAT              "Confermi formattazione?"
#define TR_INTERNALRF                  "Modulo interno"
#define TR_INTERNAL_MODULE             TR("Modulo int.", "Modulo interno")
#define TR_EXTERNAL_MODULE             TR("Modulo est.", "Modulo esterno")
#define TR_OPENTX_UPGRADE_REQUIRED     "OpenTX richiede aggiornamento"
#define TR_TELEMETRY_DISABLED          TR("Telem. off", "Telem. disabilitata")
#define TR_MORE_OPTIONS_AVAILABLE      "Altre opzioni assenti"
#define TR_NO_MODULE_INFORMATION       "Nessuna info del modulo"
#define TR_EXTERNALRF                  "Modulo esterno"
#define TR_FAILSAFE                     TR(INDENT "Failsafe", INDENT "Modo failsafe")
#define TR_FAILSAFESET                  TR(INDENT "FAILSAFE", INDENT "IMPOSTAZIONI FAILSAFE")
#define TR_REG_ID                      "Reg. ID"
#define TR_OWNER_ID                    "Owner ID"
#define TR_PINMAPSET                   "PINMAP"
#define TR_HOLD                        "Hold"
#define TR_HOLD_UPPERCASE              "HOLD"
#define TR_NONE                        "None"
#define TR_NONE_UPPERCASE              "NONE"
#define TR_MENUSENSOR                  "SENSORE"
#define TR_POWERMETER_PEAK             "Picco"
#define TR_POWERMETER_POWER            "Potenza"
#define TR_POWERMETER_ATTN             "Att."
#define TR_POWERMETER_FREQ             "Freq."
#define TR_MENUTOOLS                   "TOOLS"
#define TR_TURN_OFF_RECEIVER           "Spegni la RX"
#define TR_STOPPING                    "Fermando..."
#define TR_MENU_SPECTRUM_ANALYSER      "ANALIZZATORE SPETTRO"
#define TR_MENU_POWER_METER            "MISURATORE POTENZA"
#define TR_SENSOR              "SENSORE"
#define TR_COUNTRYCODE         TR("Cod. Paese", "Codice del Paese")
#define TR_USBMODE             "Modo USB"
#define TR_JACKMODE            "Modo JACK"
#define TR_VOICELANG           "Lingua vocale"
#define TR_UNITSSYSTEM         "Unit\200"
#define TR_EDIT                "Modifica"
#define TR_INSERT_BEFORE       "Inserisci prima"
#define TR_INSERT_AFTER        "Inserisci dopo"
#define TR_COPY                "Copia"
#define TR_MOVE                "Sposta"
#define TR_PASTE               "Incolla"
#define TR_DELETE              "Elimina"
#define TR_INSERT              TR("Inser.","Inserisci")
#define TR_RESET_FLIGHT        "Azzera volo"
#define TR_RESET_TIMER1        "Azzera Timer1"
#define TR_RESET_TIMER2        "Azzera Timer2"
#define TR_RESET_TIMER3        "Azzera Timer3"
#define TR_RESET_TELEMETRY     "Azzera Telemetria"
#define TR_STATISTICS          "Statistiche"
#define TR_ABOUT_US            "Info su"
#define TR_USB_JOYSTICK        "Joystick USB (HID)"
#define TR_USB_MASS_STORAGE    "Storage USB (SD)"
#define TR_USB_SERIAL          "Seriale USB (Debug)"
#define TR_USB_TELEMETRY       "USB Telem mirror"
#define TR_SETUP_SCREENS       "Schermate conf."
#define TR_MONITOR_SCREENS     "Monitors"
#define TR_AND_SWITCH          "Inter. AND"
#define TR_SF                  "CF"
#define TR_GF                  "GF"
#define TR_SPEAKER             INDENT "Speaker"
#define TR_BUZZER              INDENT "Buzzer"
#define TR_BYTES               "Bytes"
#define TR_MODULE_BIND         TR("[Bnd]","[Bind]")
#define TR_POWERMETER_ATTN_NEEDED      "Attenuatore necessario"
#define TR_PXX2_SELECT_RX              "Seleziona RX..."
#define TR_PXX2_DEFAULT                "<default>"
#define TR_BT_SELECT_DEVICE            "Seleziona dispositivo"
#define TR_DISCOVER             "Cerca"
#define TR_BUTTON_INIT                 BUTTON("Init")
#define TR_WAITING                     "In attesa..."
#define TR_RECEIVER_DELETE             "Cancello RX?"
#define TR_RECEIVER_RESET              "Resetto RX?"
#define TR_SHARE                       "Condividere"
#define TR_BIND                        "Bind"
#define TR_REGISTER                    TR("Reg", "Registrare")
#define TR_MODULE_RANGE                TR("[Rng]","[Range]")
#define TR_RECEIVER_OPTIONS            TR("OPZIONI RX", "OPZIONI RICEVENTE")
#define TR_DEL_BUTTON                  BUTTON(TR("Canc.", "Cancella"))
#define TR_RESET_BTN           "[Reset]"
#define TR_DEBUG                       "Debug"
#define TR_KEYS_BTN                BUTTON(TR("SW", "Tasti"))
#define TR_ANALOGS_BTN             BUTTON("Analogici")
#define TR_TOUCH_NOTFOUND          "Schermo touch non trovato"
#define TR_TOUCH_EXIT              "Tocca lo schermo per uscire"
#define TR_CALIBRATION             TR("Calibraz.", "Calibrazione")
#define TR_SET                 "[Set]"
#define TR_TRAINER             TR("Trainer", "Maestro/Allievo")
#define TR_CHANS                       "Canali"
#define TR_ANTENNAPROBLEM      CENTER "Problemi antenna TX!"
#define TR_MODELIDUSED         TR("ID gi\200 usato", "ID Modello gi\200 usato")
#define TR_MODULE              "Modulo"
#define TR_RX_NAME                     "Nome RX"
#define TR_TELEMETRY_TYPE      "Tipo Telemetria"
#define TR_TELEMETRY_SENSORS   "Sensori"
#define TR_VALUE               "Valore"
#define TR_TOPLCDTIMER         "Timer LCD Sup."
#define TR_UNIT                "Unita"
#define TR_TELEMETRY_NEWSENSOR INDENT "Aggiungi nuovo sensore"
#define TR_CHANNELRANGE        TR(INDENT "Num Canali", INDENT "Numero Canali")
#define TR_AFHDS3_RX_FREQ              TR("Freq. RX", "Frequenza RX")
#define TR_AFHDS3_ONE_TO_ONE_TELEMETRY TR("Unicast/Tel.", "Unicast/Telemetria")
#define TR_AFHDS3_ONE_TO_MANY          "Multicast"
#define TR_AFHDS3_ACTUAL_POWER         TR("Pot. att.", "Potenza attuale")
#define TR_AFHDS3_POWER_SOURCE         TR("Sorg. al.", "Sorgente alimentazione")
#define TR_ANTENNACONFIRM1     "ANTENNA EST."
#if defined(PCBX12S)
#define LEN_ANTENNA_MODES      "\023"
#define TR_ANTENNA_MODES       "Interna\0           ""Chiedi\0            ""Per modello\0       ""Interna + esterna"
#else
#define LEN_ANTENNA_MODES      "\011"
#define TR_ANTENNA_MODES       "Interna\0 ""Chiedi\0  ""Per model""Esterna"
#endif
#define TR_USE_INTERNAL_ANTENNA        TR("Usa ant. int.", "Usa antenna interna")
#define TR_USE_EXTERNAL_ANTENNA        TR("Usa ant. est.", "Usa antenna esterna")
#define TR_ANTENNACONFIRM2     TR("Controlla instal.", "Controlla prima se installata!")
#define TR_MODULE_PROTOCOL_FLEX_WARN_LINE1                "Non richiede"
#define TR_MODULE_PROTOCOL_FCC_WARN_LINE1         "Richiede FCC"
#define TR_MODULE_PROTOCOL_EU_WARN_LINE1          "Richiede EU"
#define TR_MODULE_PROTOCOL_WARN_LINE2                "firmware certificato"
#define TR_LOWALARM            INDENT "Allarme Basso"
#define TR_CRITICALALARM       INDENT "Allarme Critico"
#define TR_RSSIALARM_WARN      TR(INDENT "Tel. RSSI", "TELEMETRIA RSSI")
#define TR_NO_RSSIALARM        TR("Allarmi disab.", "Allarmi telemetrici disabilitati")
#define TR_DISABLE_ALARM       TR(INDENT "Allarmi disab.", INDENT "Allarmi telemetrici disabilitati")
#define TR_ENABLE_POPUP        "Abilita Popup"
#define TR_DISABLE_POPUP       "Disabilita Popup"
#define TR_POPUP               "Popup"
#define TR_MIN                 "Min"
#define TR_MAX                 "Max"
#define TR_CURVE_PRESET        "Preimpostate..."
#define TR_PRESET              "Preimpostate"
#define TR_MIRROR              "Mirror"
#define TR_CLEAR               "Cancella"
#define TR_RESET               "Azzera"
#define TR_RESET_SUBMENU       "Azzera..."
#define TR_COUNT               "Punti"
#define TR_PT                  "pt"
#define TR_PTS                 "pti"
#define TR_SMOOTH              "Smussa"
#define TR_COPY_STICKS_TO_OFS  TR(INDENT "Cp. stick->subtrim", INDENT "Copia Sticks su Subtrim")
#define TR_COPY_MIN_MAX_TO_OUTPUTS     TR("Cpy min/max to all",  "Copy min/max/center to all outputs")
#define TR_COPY_TRIMS_TO_OFS   TR(INDENT "Cp. trim->subtrim", INDENT "Copia Trims su Subtrim")
#define TR_INCDEC              "Inc/Decrementa"
#define TR_GLOBALVAR           "Var Globale"
#define TR_MIXSOURCE           "Sorgente Mixer"
#define TR_CONSTANT            "Constante"
#define TR_PERSISTENT_MAH      INDENT "Memo mAh"
#define TR_PREFLIGHT           "Controlli Prevolo"
#define TR_CHECKLIST           INDENT "Mostra Checklist"
#define TR_FAS_OFFSET          TR(INDENT "FAS Ofs", INDENT "FAS Offset")
#define TR_AUX_SERIAL_MODE     "Porta Seriale"
#define TR_AUX2_SERIAL_MODE    "Porta Seriale 2"
#define TR_SCRIPT              "Script"
#define TR_INPUTS              "Ingresso"
#define TR_OUTPUTS             "Uscita"
#define STR_EEBACKUP           "Copiare l'EEPROM"
#define STR_FACTORYRESET       "Inizializza dati"
#define TR_CONFIRMRESET        "Resettare TUTTI i dati?"
#define TR_TOO_MANY_LUA_SCRIPTS "Troppi Scripts Lua!"
#define TR_SPORT_UPDATE_POWER_MODE     "SP Power"
#define LEN_SPORT_UPDATE_POWER_MODES   "\004"
#define TR_SPORT_UPDATE_POWER_MODES    "AUTO""ON\0 "
#define TR_NO_TELEMETRY_SCREENS        "Nessuno Schermo Telemetria"
#define TR_TOUCH_PANEL                 "Schermo touch:"

// Horus and Taranis column headers
#define TR_PHASES_HEADERS_NAME         "Nome"
#define TR_PHASES_HEADERS_SW           "Inter"
#define TR_PHASES_HEADERS_RUD_TRIM     "Trim Deriva"
#define TR_PHASES_HEADERS_ELE_TRIM     "Trim Elevatore"
#define TR_PHASES_HEADERS_THT_TRIM     "Trim Motore"
#define TR_PHASES_HEADERS_AIL_TRIM     "Trim Alettoni"
#define TR_PHASES_HEADERS_CH5_TRIM     "Trim 5"
#define TR_PHASES_HEADERS_CH6_TRIM     "Trim 6"
#define TR_PHASES_HEADERS_FAD_IN       "Dissolv. In"
#define TR_PHASES_HEADERS_FAD_OUT      "Dissolv. Out"

#define TR_LIMITS_HEADERS_NAME         "Nome"
#define TR_LIMITS_HEADERS_SUBTRIM      "Offset"
#define TR_LIMITS_HEADERS_MIN          "Min"
#define TR_LIMITS_HEADERS_MAX          "Max"
#define TR_LIMITS_HEADERS_DIRECTION    "Direzione"
#define TR_LIMITS_HEADERS_CURVE        "Curve"
#define TR_LIMITS_HEADERS_PPMCENTER    "Centro PPM"
#define TR_LIMITS_HEADERS_SUBTRIMMODE  "Simmetria"

#define TR_LSW_HEADERS_FUNCTION        "Funzione"
#define TR_LSW_HEADERS_V1              "V1"
#define TR_LSW_HEADERS_V2              "V2"
#define TR_LSW_HEADERS_ANDSW           "Inter. AND"
#define TR_LSW_HEADERS_DURATION        "Durata"
#define TR_LSW_HEADERS_DELAY           "Ritardo"

#define TR_GVAR_HEADERS_NAME          "Nome"
#define TR_GVAR_HEADERS_FM0           "Valore su FM0"
#define TR_GVAR_HEADERS_FM1           "Valore su FM1"
#define TR_GVAR_HEADERS_FM2           "Valore su FM2"
#define TR_GVAR_HEADERS_FM3           "Valore su FM3"
#define TR_GVAR_HEADERS_FM4           "Valore su FM4"
#define TR_GVAR_HEADERS_FM5           "Valore su FM5"
#define TR_GVAR_HEADERS_FM6           "Valore su FM6"
#define TR_GVAR_HEADERS_FM7           "Valore su FM7"
#define TR_GVAR_HEADERS_FM8           "Valore su FM8"

// Horus footer descriptions
#define TR_LSW_DESCRIPTIONS    { "Tipo o funzione di confronto", "Prima variabile", "Seconda variabile o costante", "Seconda variabile o costante", "Condizione aggiuntiva da abilitare linea", "Durata minima ON dell'interruttore logico", "Durata minima TRUE affinché l'interruttore si accenda" }

// Horus layouts and widgets
#define TR_FIRST_CHANNEL               "Primo canale"
#define TR_FILL_BACKGROUND             "Riempi lo sfondo?"
#define TR_BG_COLOR                    "Colore sfondo"
#define TR_SLIDERS_TRIMS               "Sliders+Trims"
#define TR_SLIDERS                     "Sliders"
#define TR_FLIGHT_MODE                 "Modo di volo"
#define TR_INVALID_FILE                "File invalido"
#define TR_TIMER_SOURCE                "Sorgente timer"
#define TR_SIZE                        "Dimensione"
#define TR_SHADOW                      "Ombra"
#define TR_TEXT                        "Testo"
#define TR_COLOR                       "Colore"
#define TR_MAIN_VIEW_X                 "Vista principale X"
#define TR_PANEL1_BACKGROUND           "Sfondo Panello 1"
#define TR_PANEL2_BACKGROUND           "Sfondo Panello 2"

// About screen
#define TR_ABOUTUS             TR(" INFO ", "INFO SU")

#define TR_ABOUT_OPENTX_1      TR("OpenTX\001e'\001open\001source,\001non", "OpenTX e' open source, non-")
#define TR_ABOUT_OPENTX_2      TR("commercial,\001wo\001warranties.", "commerciale, fornito senza")
#define TR_ABOUT_OPENTX_3      TR("It\001was\001developed\001for\001free.", "garanzie. E' stato sviluppato")
#define TR_ABOUT_OPENTX_4      TR("Support through donations", "gratuitamente. Il supporto")
#define TR_ABOUT_OPENTX_5      TR("is welcome!", "mediante donazioni e' gradito!")

#define TR_ABOUT_BERTRAND_1    "Bertrand Songis"
#define TR_ABOUT_BERTRAND_2    "Autore principale OpenTX"
#define TR_ABOUT_BERTRAND_3    "Co-autore Companion"

#define TR_ABOUT_MIKE_1        "Mike Blandford"
#define TR_ABOUT_MIKE_2        "Guru codice e driver"
#define TR_ABOUT_MIKE_3        "esperto a basso livello"
#define TR_ABOUT_MIKE_4        "Ispiratore"

#define TR_ABOUT_ROMOLO_1      "Romolo Manfredini"
#define TR_ABOUT_ROMOLO_2      "Autore Companion"
#define TR_ABOUT_ROMOLO_3      ""

#define TR_ABOUT_ANDRE_1       "Andre Bernet"
#define TR_ABOUT_ANDRE_2       "Funzionalit\200, usabilit\200,"
#define TR_ABOUT_ANDRE_3       "debugging, documentazione"

#define TR_ABOUT_ROB_1         "Rob Thomson"
#define TR_ABOUT_ROB_2         "Webmaster openrcforums"

#define TR_ABOUT_KJELL_1       "Kjell Kernen"
#define TR_ABOUT_KJELL_2       "www.open-tx.org main author"
#define TR_ABOUT_KJELL_3       "OpenTX Recorder author"
#define TR_ABOUT_KJELL_4       "Companion contributor"

#define TR_ABOUT_MARTIN_1      "Martin Hotar"
#define TR_ABOUT_MARTIN_2      "Graphic Designer"

#if defined(MANUFACTURER_FRSKY)
  #define TR_ABOUT_HARDWARE_1  "FrSky"
  #define TR_ABOUT_HARDWARE_2  "Disegno/produzione Hardware"
  #define TR_ABOUT_HARDWARE_3  "Contributi al Firmware"
#elif defined(MANUFACTURER_RADIOMASTER)
  #define TR_ABOUT_HARDWARE_1  "Radiomaster"
  #define TR_ABOUT_HARDWARE_2  "Disegno/produzione Hardware"
  #define TR_ABOUT_HARDWARE_3  "Contributi al Firmware"
#elif defined(MANUFACTURER_JUMPER)
  #define TR_ABOUT_HARDWARE_1  "JumperRC"
  #define TR_ABOUT_HARDWARE_2  "Disegno/produzione Hardware"
  #define TR_ABOUT_HARDWARE_3  "Contributi al Firmware"
#else
  #define TR_ABOUT_HARDWARE_1  "Brent Nelson"
  #define TR_ABOUT_HARDWARE_2  "Designer/Produttore SKY9X"
  #define TR_ABOUT_HARDWARE_3  ""
#endif

#define TR_ABOUT_PARENTS_1     "Progetti correlati"
#define TR_ABOUT_PARENTS_2     "ersky9x (Mike Blandford)"
#define TR_ABOUT_PARENTS_3     "ER9X (Erez Raviv)"
#define TR_ABOUT_PARENTS_4     "TH9X (Thomas Husterer)"

#define TR_CHR_SHORT  's'
#define TR_CHR_LONG   'l'
#define TR_CHR_TOGGLE 't'
#define TR_CHR_HOUR   'h'
#define TR_CHR_INPUT  'I'   // Values between A-I will work

#define TR_BEEP_VOLUME         "Volume Beep"
#define TR_WAV_VOLUME          "Volume Wav"
#define TR_BG_VOLUME           "Volume Sf"

#define TR_TOP_BAR             "Barra sup."
#define TR_FLASH_ERASE                 "Cancello flash..."
#define TR_FLASH_WRITE                 "Scrivo flash..."
#define TR_OTA_UPDATE                  "Aggiorn. OTA..."
#define TR_MODULE_RESET                "Reset modulo..."
#define TR_UNKNOWN_RX                  "RX sconociuta"
#define TR_UNSUPPORTED_RX              "RX non compatibile"
#define TR_OTA_UPDATE_ERROR            "Errore agg. OTA"
#define TR_DEVICE_RESET                "Resetto dispositivo..."
#define TR_ALTITUDE            INDENT "Altitudine"
#define TR_SCALE               "Scala"
#define TR_VIEW_CHANNELS       "Vedi Canali"
#define TR_VIEW_NOTES          "Vedi Note"
#define TR_MODEL_SELECT        "Seleziona Modello"
#define TR_MODS_FORBIDDEN      "Modifica proibita!"
#define TR_UNLOCKED            "Sbloccato"
#define TR_ID                  "ID"
#define TR_PRECISION           "Precisione"
#define TR_RATIO               "Ratio"
#define TR_FORMULA             "Formula"
#define TR_CELLINDEX           "Indice cella"
#define TR_LOGS                "Logs"
#define TR_OPTIONS             "Opzioni"
#define TR_FIRMWARE_OPTIONS    "Opzioni firmware"

#define TR_ALTSENSOR           "Sensore Alt"
#define TR_CELLSENSOR          "Sensore Cell"
#define TR_GPSSENSOR           "Sensore GPS"
#define TR_CURRENTSENSOR       "Sensore"
#define TR_AUTOOFFSET          "Offset Auto"
#define TR_ONLYPOSITIVE        "Positivo"
#define TR_FILTER              "Filtro"
#define TR_TELEMETRYFULL       "Tutti gli slot sono pieni!"
#define TR_SERVOS_OK           "Servi OK"
#define TR_SERVOS_KO           "Servi KO"
//TODO: translation
#define TR_INVERTED_SERIAL     INDENT "Invert."
#define TR_IGNORE_INSTANCE     TR(INDENT "No inst.", INDENT "Ignora instanza")
#define TR_DISCOVER_SENSORS    "Cerca nuovi sensori"
#define TR_STOP_DISCOVER_SENSORS "Ferma ricerca"
#define TR_DELETE_ALL_SENSORS  "Elimina tutti i sensori"
#define TR_CONFIRMDELETE       "Confermi " LCDW_128_480_LINEBREAK "eliminazione?"
#define TR_SELECT_WIDGET       "Seleziona widget"
#define TR_REMOVE_WIDGET       "Rimuovi widget"
#define TR_WIDGET_SETTINGS     "Settaggio widget"
#define TR_REMOVE_SCREEN       "Rimuovi schermo"
#define TR_SETUP_WIDGETS       "Setta widgets"
#define TR_USER_INTERFACE      "Interfaccia utente"
#define TR_THEME               "Tema"
#define TR_SETUP               "Imposta"
#define TR_MAINVIEWX           "Vista princ. X"
#define TR_LAYOUT              "Layout"
#define TR_ADDMAINVIEW         "Aggiungi vista princ."
#define TR_BACKGROUND_COLOR    "Colore background"
#define TR_MAIN_COLOR          "Colore principale"
#define TR_BAR2_COLOR                  "Secondary bar color"
#define TR_BAR1_COLOR                  "Main bar color"
#define TR_TEXT_COLOR                  "Text color"
#define TR_TEXT_VIEWER         "Visualizzatore testi"

#define TR_MENU_INPUTS         "\314Ingressi"
#define TR_MENU_LUA            "\322Lua scripts"
#define TR_MENU_STICKS         "\307Sticks"
#define TR_MENU_POTS           "\310Pots"
#define TR_MENU_MAX            "\315MAX"
#define TR_MENU_HELI           "\316Ciclico"
#define TR_MENU_TRIMS          "\313Trims"
#define TR_MENU_SWITCHES       "\312Interrut."
#define TR_MENU_LOGICAL_SWITCHES "\312Interrut. Logici"
#define TR_MENU_TRAINER        "\317Trainer"
#define TR_MENU_CHANNELS       "\320Canali"
#define TR_MENU_GVARS          "\311GVars"
#define TR_MENU_TELEMETRY      "\321Telemetry"
#define TR_MENU_DISPLAY        "DISPLAY"
#define TR_MENU_OTHER          "Altro"
#define TR_MENU_INVERT         "Inverti"
#define TR_JITTER_FILTER       "Filtro ADC"
#define TR_RTC_CHECK           TR("Controllo RTC", "Controllo volt. RTC")
#define TR_AUTH_FAILURE        "Auth-failure"
#define TR_RACING_MODE         "Racing mode"

#define ZSTR_VFR               "\026\006\022"
#define ZSTR_RSSI              "\022\023\023\011"
#define ZSTR_R9PW              "\022\044\020\027"
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
#define ZSTR_GPSDATETIME       "\004\377\354\377"
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

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

// PL translations author: Jacek Drewniak <jpdjpd33@gmail.com>

/*
 * !!!!! DO NOT EDIT pl.h - EDIT pl.h.txt INSTEAD !!!!!!!
 *
 * In order to make translations easier pl.h.txt is parsed and national
 * characters are replaced by bitmap codes. The result is pl.h.
 * See translate.py in the util folder for the list of character codes
 *
 * !!!!! DO NOT EDIT pl.h - EDIT pl.h.txt INSTEAD !!!!!!!
 *
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
#define TR_OFFON               "WY\214""ZA\214"

#define LEN_MMMINV             "\003"
#define TR_MMMINV              "---""REV"

#define LEN_VBEEPMODE          "\005"
#define TR_VBEEPMODE           "Cichy""Alarm""BezPr""Wsz.\0"

#define LEN_VBLMODE            TR("\004", "\010")
#define TR_VBLMODE             TR("Wy\203\0""Przy""Dr\200\207""Oba\0""Za\203\0", "Wy\203\200cz  ""Przycisk""Dr\200zki\0 ""Oba\0    ""W\203\200cz   ")

#define LEN_TRNMODE            "\003"
#define TR_TRNMODE             "Wy\203"" +="" :="

#define LEN_TRNCHN             "\003"
#define TR_TRNCHN              "KN1KN2KN3KN4"

#define LEN_AUX_SERIAL_MODES   "\015"  /*13 decimal*/
#if defined(CLI) || defined(DEBUG)
#define TR_AUX_SERIAL_MODES    "Debug\0       ""S-Port Kopia ""Telemetria\0  ""Trener SBUS\0 ""LUA\0          "
#else
#define TR_AUX_SERIAL_MODES    "Wy\203\200cz\0      ""S-Port Kopia ""Telemetria\0  ""Trener SBUS\0 ""LUA\0          "
#endif

#define LEN_SWTYPES            "\006"
#define TR_SWTYPES             "Brak\0 ""Chwil.""2POZ\0 ""3POZ\0"

#define LEN_POTTYPES           TR("\013","\017") /*15 decimal*/
#define TR_POTTYPES            TR("None\0      ""Pot w. det\0""Multipos\0  ""Pot\0       ", "Brak\0          ""Poten z zapadk\200""Prze\203.Wielopoz.""Potencjometr\0  ")

#define LEN_SLIDERTYPES        "\006"
#define TR_SLIDERTYPES         "Brak\0 ""Suwak "

#define LEN_VLCD               "\006"
#define TR_VLCD                "NormalOptrex"

#define LEN_VPERSISTENT        "\014"  /*12 decimal*/
#define TR_VPERSISTENT         "Wy\203\200cz\0     ""Lot\0        ""R\202czny Reset"

#define LEN_COUNTRYCODES       TR("\002", "\007")
#define TR_COUNTRYCODES        TR("US""JP""EU", "Ameryka""Japonia""Europa\0")

#if defined(RADIO_FAMILY_TBS)
#define LEN_USBMODES                   TR("\006", "\010")
#define TR_USBMODES                    TR("Ask\0  ""Joyst\0""Agent\0""SDCard""Serial", "Ask\0    ""Joystick""Agent\0 ""Storage\0""Serial\0 ")
#else
#define LEN_USBMODES           TR("\006", "\010")
#if defined(DEBUG)
#define TR_USBMODES            TR("Ask\0  ""Joyst\0""SDCard""Serial", "Ask\0    ""Joystick""Storage\0""Serial\0 ")
#else
#define TR_USBMODES            TR("Ask\0  ""Joyst\0""SDCard""Telem\0", "Ask\0    ""Joystick""Storage\0""Telem\0  ")
#endif
#endif

#define LEN_JACKMODES                  "\007"
#define TR_JACKMODES                   "Ask\0   ""Audio\0 ""Trainer"

#define LEN_TELEMETRY_PROTOCOLS "\017"
#define TR_TELEMETRY_PROTOCOLS "FrSky S.PORT\0  ""FrSky D\0       ""FrSky D (cable)""TBS Crossfire\0 ""Spektrum\0      ""AFHDS2A IBUS\0  ""Multi Telemetry"

#define TR_MULTI_CUSTOM        "Custom"

#define LEN_VTRIMINC           TR("\006", "\013") /*11 decimal*/
#define TR_VTRIMINC            TR("Expo\0 ""B.Dok\203""Dok\203.\0""\217redni""Zgrubn", "Expotencja ""B.Dok\203adny\0""Dok\203adny\0  ""\217redni\0    ""Zgrubny\0   ")


#define LEN_VDISPLAYTRIMS      "\006"
#define TR_VDISPLAYTRIMS       "Nie\0  ""Zmie\204\0""Tak\0"

#define LEN_VBEEPCOUNTDOWN     "\006"
#define TR_VBEEPCOUNTDOWN      "Cichy Pik  \0D\210wi\202kWibrac"

#define LEN_VVARIOCENTER       "\006"
#define TR_VVARIOCENTER        "Ton\0  ""Cicho "

#define LEN_CURVE_TYPES        "\010" /*8 decimal*/
#define TR_CURVE_TYPES         "Standard""W\203asny\0 "

#define LEN_RETA123            "\001"

#if defined(PCBHORUS)
  #define TR_RETA123           "KWGL13245LR"
#elif defined(PCBX9E)
  #define TR_RETA123           "KWGL1234LRLR"
#elif defined(PCBTARANIS) || defined(REVX)
  #define TR_RETA123           "KWGL123LR"
#elif defined(PCBSKY9X)
    #define TR_RETA123         "KWGL123a"
#else
  #define TR_RETA123           "KWGL123"
#endif

#if defined(PCBSKY9X) && defined(REVX)
  #define LEN_VOUTPUT_TYPE     "\011"
  #define TR_VOUTPUT_TYPE      "OpenDrain""PushPull\0"
#endif

#define LEN_VCURVEFUNC         "\003"
#define TR_VCURVEFUNC          "---""x>0""x<0""|x|""f>0""f<0""|f|"

#define LEN_VMLTPX             "\010" /*8 decimal*/
#define TR_VMLTPX              "Dodaj\0  ""Mn\205\207    ""Zast\200p\0 "

#define LEN_VMLTPX2            "\002"
#define TR_VMLTPX2             "+=""*="":="

#define LEN_VMIXTRIMS          "\003"

#if defined(PCBHORUS)
  #define TR_VMIXTRIMS         "WY\214""ZA\214""SK\0""SW\0""GAZ""Lot""Ail""T5\0""T6\0"
#else
  #define TR_VMIXTRIMS         "WY\214""ZA\214""SK\0""SW\0""GAZ""Lot"
#endif

#if LCD_W >= 212
  #define TR_CSWTIMER          "Timer"
  #define TR_CSWSTICKY         "Sta\203y"
  #define TR_CSWRANGE          "Zasi\202"
  #define TR_CSWSTAY           "Brzeg"
#else
  #define TR_CSWTIMER          "Tim\0 "
  #define TR_CSWSTICKY         "Sta\203y"
    #define TR_CSWRANGE        "Zasi\202"
    #define TR_CSWSTAY         "Brzeg"
#endif

  #define TR_CSWEQUAL          "a=x\0 "

#define LEN_VCSWFUNC           "\005"
#define TR_VCSWFUNC            "---\0 " TR_CSWEQUAL "a\173x\0 ""a>x\0 ""a<x\0 " TR_CSWRANGE "|a|>x""|a|<x""AND\0 ""OR\0  ""XOR\0 " TR_CSWSTAY "a=b\0 ""a>b\0 ""a<b\0 ""\306}x\0 ""|\306|}x" TR_CSWTIMER TR_CSWSTICKY

#define LEN_VFSWFUNC           "\012" /*10 decimal*/

#if defined(VARIO)
  #define TR_VVARIO            "Wario\0    "
#else
  #define TR_VVARIO            "[Wario]\0  "
#endif

#if defined(AUDIO)
  #define TR_SOUND             "GrajD\210wi\202k"
#else
  #define TR_SOUND             "Pikanie\0  "
#endif

#if defined(HAPTIC)
  #define TR_HAPTIC            "Wibracje\0 "
#else
  #define TR_HAPTIC            "[Wibracje]"
#endif

#if defined(VOICE)
  #if defined(PCBSKY9X)
    #define TR_PLAY_TRACK      "\217cie\207ka\0  "
  #else
    #define TR_PLAY_TRACK      "Graj\217cie\207k"
  #endif
  #define TR_PLAY_BOTH         "Graj Oba\0 "
  #define TR_PLAY_VALUE        TR("Graj Wart\0","GrajWarto\206")
#else
  #define TR_PLAY_TRACK        "[Graj \217ci]"
  #define TR_PLAY_BOTH         "[Graj Oba]"
  #define TR_PLAY_VALUE        "[Graj War]"
#endif

#define TR_SF_BG_MUSIC        "Muz. t\203a\0 ""Muz.t\203a ||"

#if defined(SDCARD)
  #define TR_SDCLOGS           "Logi->SD\0 "
#else
  #define TR_SDCLOGS           "[Logowa\201]\0"
#endif

#if defined(GVARS)
  #define TR_ADJUST_GVAR       "Ustaw\0    "
#else
  #define TR_ADJUST_GVAR       "[UstawZG] "
#endif

#if defined(LUA)
  #define TR_SF_PLAY_SCRIPT   "SkryptyLua"
#else
  #define TR_SF_PLAY_SCRIPT   "[Lua]\0    "
#endif

#if defined(DEBUG)
  #define TR_SF_TEST          "Test\0"
#else
  #define TR_SF_TEST
#endif

#if defined(OVERRIDE_CHANNEL_FUNCTION) && LCD_W >= 212
  #define TR_SF_SAFETY        "Bezpiecz\0 "
#elif defined(OVERRIDE_CHANNEL_FUNCTION)
  #define TR_SF_SAFETY        "Bezp.\0    "
#else
  #define TR_SF_SAFETY        "---\0      "
#endif

#define TR_SF_SCREENSHOT      "Zrzut Ekra"
#define TR_SF_RACING_MODE     "RacingMode"
#define TR_SF_RESERVE         "[rezerwa]\0"

#define TR_VFSWFUNC            TR_SF_SAFETY "Trener \0  ""Inst-Trim ""Resetuj\0  ""Ustaw\0    " TR_ADJUST_GVAR "G\203o\206no\206\201\0 " "SetFailsfe" "RangeCheck" "ModuleBind" TR_SOUND TR_PLAY_TRACK TR_PLAY_VALUE TR_SF_RESERVE TR_SF_PLAY_SCRIPT TR_SF_RESERVE TR_SF_BG_MUSIC TR_VVARIO TR_HAPTIC TR_SDCLOGS "Pod\206wietl\0" TR_SF_SCREENSHOT TR_SF_RACING_MODE TR_SF_TEST

#define LEN_VFSWRESET          TR("\004", "\011")  /*9 decimal*/

#define TR_FSW_RESET_TELEM   TR("Telm", "Telemetra")

#if LCD_W >= 212
  #define TR_FSW_RESET_TIMERS  "Timer 1\0 ""Timer 2\0 ""Timer 3\0 "
#else
  #define TR_FSW_RESET_TIMERS  "Tmr1""Tmr2""Tmr3"
#endif

#define TR_VFSWRESET           TR(TR_FSW_RESET_TIMERS "All\0" TR_FSW_RESET_TELEM, TR_FSW_RESET_TIMERS "All\0     " TR_FSW_RESET_TELEM)

#define LEN_FUNCSOUNDS         TR("\004", "\006")
#define TR_FUNCSOUNDS          TR("Bp1\0""Bp2\0""Bp3\0""Ost1""Ost2""Chee""Rata""Tik ""Syre""Dzwo""SciF""Robt""Chrp""Tada""Crck""Alrm", "Beep1 ""Beep2 ""Beep3 ""Ostrz1""Ostrz1""Cheep ""Ratata""Tick  ""Syrena""Dzwone""SciFi ""Robot ""Chirp ""Tada  ""Krytcz""AlmZeg")

#define LEN_VTELEMCHNS         "\004"

  #define TR_TELEM_RESERVE     TR("[--]", "[---]")
  #define TR_TELEM_TIME        TR("Time", "Time\0")
  #define TR_RAS               TR("SWR\0", "SWR\0 ")
  #define TR_RX_BATT           TR("[NA]", "[NA]\0")
  #define TR_A3_A4             TR("A3\0 ""A4\0 ", "A3\0  ""A4\0  ")
  #define TR_A3_A4_MIN         TR("A3-\0""A4-\0", "A3-\0 ""A4-\0 ")

#define TR_ASPD_MAX            TR("ASp+", "ASpd+")

#if LCD_W >= 212
  #define TR_TELEM_RSSI_RX     "RSSI\0"
#else
  #define TR_TELEM_RSSI_RX     TR("Rx\0  ", "Rx\0 ")
#endif

  #define TR_TELEM_TIMERS      TR("Tmr1""Tmr2""Tmr3", "Tmr1\0""Tmr2\0""Tmr3\0")

#define LENGTH_UNIT_IMP        "ft\0"
#define SPEED_UNIT_IMP         "mph"
#define LENGTH_UNIT_METR       "m\0 "
#define SPEED_UNIT_METR        "kmh"

#define LEN_VUNITSSYSTEM     TR("\006", "\010") /*8 decimal*/
#define TR_VUNITSSYSTEM      TR("Metr. ""Imper.", "Metryczn""Imperial")
#define LEN_VTELEMUNIT       "\003"
#define TR_VTELEMUNIT        "-\0 ""V\0 ""A\0 ""mA\0""kts""m/s""f/s""kmh""mph""m\0 ""ft\0""@C\0""@F\0""%\0 ""mAh""W\0 ""mW\0""dB\0""rpm""g\0 ""@\0 ""rad""ml\0""fOz""mlm""Hz\0""mS\0""uS\0""km\0""dbm"

#define STR_V                  (STR_VTELEMUNIT+1)
#define STR_A                  (STR_VTELEMUNIT+4)

#define LEN_VTELEMSCREENTYPE   "\006"
#define TR_VTELEMSCREENTYPE    "Brak\0 ""Liczb\0""Paski\0""Skrypt"

#define LEN_GPSFORMAT          "\004"
#define TR_GPSFORMAT           "DMS\0""NMEA"

#define LEN2_VTEMPLATES        13
#define LEN_VTEMPLATES         "\015" /*13 decimal*/
#define TR_TEMPLATE_CLEAR_MIXES        "Usu\204 Miksy   "
#define TR_TEMPLATE_SIMPLE_4CH         "Prosty. 4kn\0 "
#define TR_TEMPLATE_STICKY_TCUT        "Prze\203\200-T-Cut\0"
#define TR_TEMPLATE_VTAIL              "V-Tail       "
#define TR_TEMPLATE_DELTA              "Elevon\\Delta "
#define TR_TEMPLATE_ECCPM              "eCCPM        "
#define TR_TEMPLATE_HELI               "Ustaw Heli  \0"
#define TR_TEMPLATE_SERVO_TEST         "Test serwa  \0"

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

#define TR_STICKS_VSRCRAW      "\307SK\0""\307SW\0""\307Gaz""\307Lot"

#if defined(PCBHORUS)
  #define TR_TRIMS_VSRCRAW     "\313SK\0""\313SW\0""\313Gaz""\313Lot""\313T5\0""\313T6\0"
#else
  #define TR_TRIMS_VSRCRAW     TR("TrSK""TrSW""TrGA""TrLO", "\313SK ""\313SW ""\313Gaz""\313Lot")
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
#define TR_VTMRMODES           "WY\214""ABS""THs""TH%""THt"

#define LEN_VTRAINERMODES              "\022" /*18 decimal*/
#define TR_VTRAINER_MASTER_JACK        "Trener/Jack\0      "
#define TR_VTRAINER_SLAVE_JACK         "Ucze\204/Jack\0       "
#define TR_VTRAINER_MASTER_SBUS_MODULE "Trener/SBUS Modu\203 "
#define TR_VTRAINER_MASTER_CPPM_MODULE "Trener/CPPM Modu\203 "
#define TR_VTRAINER_MASTER_BATTERY     "Trener/Serial\0    "
#define TR_VTRAINER_BLUETOOTH          TR("Master/BT\0        ""Slave/BT\0", "Master/Bluetooth\0 ""Slave/Bluetooth\0  ")
#define TR_VTRAINER_MULTI              "Master/Multi\0     "
#define TR_VTRAINER_SPORT_SBUS         "Master/SBUS-Sport\0"

#define LEN_VFAILSAFE          "\011" /*9 decimal*/
#define TR_VFAILSAFE           "Brak   \0 ""Utrzymuj\0""W\203asne  \0""0 sygna\203u""Odbiornik"


#define LEN_VSENSORTYPES        "\012" /*10 decimal*/
#define TR_VSENSORTYPES        "U\207ytkownik""Obliczone "

#define LEN_VFORMULAS          "\010" /*8 decimal*/
#define TR_VFORMULAS           "Dodaj\0  ""\217rednie\0""Min\0    ""Max\0    ""Mn\205\207\0   ""Zliczani""Kom\205rka\0""Zu\207ycie\0""Zasi\202g\0 "

#define LEN_VPREC              "\004"
#define TR_VPREC               "0.--""0.0 ""0.00"

#define LEN_VCELLINDEX         "\007"
#define TR_VCELLINDEX          "Niskie\0""1\0     ""2\0     ""3\0     ""4\0     ""5\0     ""6\0     ""Wysokie""Delta\0"

#define LEN_GYROS                      "\004"
#define TR_GYROS                       "GyrX""GyrY"

#define LEN_TEXT_SIZE          "\010"
#define TR_TEXT_SIZE           "Standard""Tiny\0   ""Small\0  ""Mid\0    ""Double\0 "

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

#define TR_MENUWHENDONE        CENTER "\006" TR_ENTER " > DALEJ"
#define TR_FREE                "woln"
#define TR_DELETEMODEL         "SKASUJ MODEL"
#define TR_COPYINGMODEL        "Kopiuj model.."
#define TR_MOVINGMODEL         "Przenie\206 model."
#define TR_LOADINGMODEL        "Za\203aduj model.."
#define TR_NAME                "Nazwa"
#define TR_MODELNAME           "Nazwa modelu"
#define TR_PHASENAME           "Nazwa Fazy"
#define TR_MIXNAME             "Nazwa Mix"
#define TR_INPUTNAME           "Wpisz Nazw"
  #define TR_EXPONAME          "LiniaNazw"
#define TR_BITMAP              "Obrazek    "
#define TR_TIMER               TR("Timer", "Timer ")
#define TR_ELIMITS             TR("Limi+25%", "Kana\203 +/- 125% ")
#define TR_ETRIMS              TR("E.Trym ", "Poszer Trymery")
#define TR_TRIMINC             "Krok Trym"
#define TR_DISPLAY_TRIMS       TR("Pok.Trym.","Poka\207 Trymery")
#define TR_TTRACE              TR("\221r\205d gaz", INDENT "\221r\205d\203o gazu ")
#define TR_TTRIM               TR("TryGaz", INDENT "TrymWolnObrot\205")
#define TR_TTRIM_SW            TR("T-Trim-Sw", INDENT "Trim switch")
#define TR_BEEPCTR             TR("\217rodBeep", "Pikn.\217rodka")
#define TR_USE_GLOBAL_FUNCS    TR("Funk.Glb.","U\207yj Funkcji Glb")
#if defined(PCBSKY9X) && defined(REVX)
  #define TR_OUTPUT_TYPE       INDENT "Wyj\206cie"
#endif
#define TR_PROTOCOL            TR("Proto", "Protok\205\203")
#define TR_PPMFRAME            INDENT "Ramka PPM"
#define TR_REFRESHRATE               TR(INDENT "Refresh", INDENT "Refresh rate")
#define STR_WARN_BATTVOLTAGE           TR(INDENT "Output is VBAT: ", INDENT "Warning: output level is VBAT: ")
#define TR_WARN_5VOLTS                 "Warning: output level is 5 volts"
#define TR_MS                  "ms"
#define TR_FREQUENCY                   INDENT "Frequency"
#define TR_SWITCH              "Prze\203\200"
#define TR_TRIMS               "Trymy"
#define TR_FADEIN              "Pojawia"
#define TR_FADEOUT             "Zanik   "
#define TR_DEFAULT             "(Bazowa) "
#define TR_CHECKTRIMS          CENTER "\006Spr  \012Trymy"
#define OFS_CHECKTRIMS         CENTER_OFS+(9*FW)
#define TR_SWASHTYPE           "Typ tarczy"
#define TR_COLLECTIVE          TR("Kolektyw", "\221r\205d\203o Kolektywu")
#define TR_AILERON             TR("Lateral cyc.", "Lateral cyc. source")
#define TR_ELEVATOR            TR("Long. cyc.", "Long. cyc. source")
#define TR_SWASHRING           "Tarcza    "
#define TR_ELEDIRECTION        TR("Ster Wysoko\206c","D\203ug. Kierunku cyklu")
#define TR_AILDIRECTION        TR("Kierunek Lotk","Lateral cyc. direction")
#define TR_COLDIRECTION        TR("PIT Direction","Coll. pitch direction")
#define TR_MODE                "Tryb"
#define TR_SUBTYPE             INDENT "Subtype"
#define TR_NOFREEEXPO          "BrakWoln.Expo!"
#define TR_NOFREEMIXER         "BrakWoln.Mix\205w!"
#define TR_SOURCE               "\221r\205d\203o"
#define TR_WEIGHT              "Waga "
#define TR_EXPO                TR("Expo", "Exponential")
#define TR_SIDE                "Strona"
#define TR_DIFFERENTIAL        "R\205\207nic"
#define TR_OFFSET              "Ofset "
#define TR_TRIM                "Trym"
#define TR_DREX                "DRex"
#define DREX_CHBOX_OFFSET      30
#define TR_CURVE               "Krzywa"
#define TR_FLMODE              "Tryb"
#define TR_MIXWARNING          "UWAGA"
#define TR_OFF                 "Wy\203."
#define TR_ANTENNA                     "Antenna"
#define TR_NO_INFORMATION              TR("No info", "No information")
#define TR_MULTPX              "Z\203\200cz."
#define TR_DELAYDOWN           "Op\205zn.(-)"
#define TR_DELAYUP             "Op\205zn.(+)"
#define TR_SLOWDOWN            "Spowoln.(-)"
#define TR_SLOWUP              "Spowoln.(+)"
#define TR_MIXES               "MIKSERY"
#define TR_CV                  "Kr"
#define TR_GV                  TR("G", "ZG")
#define TR_ACHANNEL            "A\004Kana\203"
#define TR_RANGE               INDENT "Zakres"
#define TR_CENTER              INDENT "\217rodek"
#define TR_BAR                 "Pas"
#define TR_ALARM               INDENT "Alarm"
#define TR_USRDATA             TR("Dane U\207ytk.", "Dane u\207ytkownika")
#define TR_BLADES              " \214opaty \206migla"
#define TR_SCREEN              "Ekran "
#define TR_SOUND_LABEL         "D\210wi\202k"
#define TR_LENGTH              INDENT"D\203ugo\206\201"
#define TR_BEEP_LENGTH         INDENT "D\203ug. Beepa"
#define TR_SPKRPITCH           INDENT"D\210wi\202k"
#define TR_HAPTIC_LABEL        "Wibracja"
#define TR_HAPTICSTRENGTH      INDENT"Si\203a"
#define TR_GYRO_LABEL                  "Gyro"
#define TR_GYRO_OFFSET                 "Offset"
#define TR_GYRO_MAX                    "Max"
#define TR_CONTRAST            "Kontrast"
#define TR_ALARMS_LABEL        "Alarmy"
#define TR_BATTERY_RANGE       TR("Poka\207PasBat.", "Poka\207 pasek bateri")
#define TR_BATTERYWARNING      INDENT"Bateria roz\203adowana"
#define TR_INACTIVITYALARM     INDENT"Alarm bezczynno\206ci"
#define TR_MEMORYWARNING       INDENT"Pe\203na pami\202\201"
#define TR_ALARMWARNING        INDENT"Wy\203\200cz d\210wi\202k"
#define TR_RSSISHUTDOWNALARM   TR(INDENT "Rssi Shutdown", INDENT "Check Rssi on Shutdown")
#define TR_MODEL_STILL_POWERED "Model still powered"
#define TR_MODEL_SHUTDOWN              "Shutdown ?"
#define TR_PRESS_ENTER_TO_CONFIRM      "Press enter to confirm"
#define TR_THROTTLE_LABEL      "Gaz    "
#define TR_THROTTLEREVERSE     TR("OdwrGaz", INDENT "OdwrGaz")
#define TR_MINUTEBEEP          TR("Minuta", "PikCoMinut\202")
#define TR_BEEPCOUNTDOWN       INDENT "Odliczanie"
#define TR_PERSISTENT          TR(INDENT "Dok\203.", INDENT "Dok\203adny")
#define TR_BACKLIGHT_LABEL     "Pod\206wietl"
#define TR_GHOST_MENU_LABEL            "GHOST MENU"
#define TR_STATUS                      "Status"
#define TR_RAW_12BITS                  "Raw 12 bits"
#define TR_BLDELAY             INDENT"Czas trwania"
#define TR_BLONBRIGHTNESS      INDENT"Jasno\206c w\203."
#define TR_BLOFFBRIGHTNESS     INDENT"Jasno\206\201 wy\203."
#define TR_KEYS_BACKLIGHT              "Keys backlight"
#define TR_BLCOLOR             INDENT "Color"
#define TR_SPLASHSCREEN        "Logo ekranu"
#define TR_PWR_ON_DELAY                "Pwr On delay"
#define TR_PWR_OFF_DELAY               "Pwr Off delay"
#define TR_THROTTLEWARNING     TR(INDENT"OstrzGaz", INDENT "Ostrze\207enieGaz")
#define TR_SWITCHWARNING       TR(INDENT "OstrzPrze\203", INDENT "PozycjaPrze\203")
#define TR_POTWARNINGSTATE     TR(INDENT "Pot&Slid.", INDENT "Pots & sliders")
#define TR_SLIDERWARNING       TR(INDENT "Slid. pos.", INDENT "Slider positions")
#define TR_POTWARNING          TR(INDENT "Ostrz.Poten.", INDENT "PozycjaPotencj.")
#define TR_TIMEZONE            TR("Strefa czasowa" ,"Strefa czasowa GPS")
#define TR_ADJUST_RTC          TR("RegulujRTC", INDENT "RegulujRTC")
#define TR_GPS                 "GPS"
#define TR_RXCHANNELORD        TR("KolejnKan.RX", "Bazowa kolejno\206\201 kana\203\205w")
#define TR_STICKS              "Dr\200\207ki"
#define TR_POTS                "Pots"
#define TR_SWITCHES            "Prze\203\200cz"
#define TR_SWITCHES_DELAY      "Op\205\210n.Prze\203."
#define TR_SLAVE               "Ucze\204"
#define TR_MODESRC             " Tryb\006% \221r\205d\203o"
#define TR_MULTIPLIER          "Mno\207nik"
#define TR_CAL                 "Kal."
#define TR_VTRIM               "Trym- +"
#define TR_BG                  "BG:"
#if defined(PCBHORUS)
  #define TR_MENUTOSTART       "Press [Enter] to start"
  #define TR_SETMIDPOINT       "Center sticks/pots/sliders and press [Enter]"
  #define TR_MOVESTICKSPOTS    "Move sticks, pots and sliders and press [Enter]"
#elif defined(COLORLCD)
  #define TR_MENUTOSTART       TR_ENTER " KALIBRUJ"
  #define TR_SETMIDPOINT       "Centruj dra\207ki/potencj"
  #define TR_MOVESTICKSPOTS    "Poruszaj dr\200\207kami/pot"
#else
  #define TR_MENUTOSTART       CENTER "\010" TR_ENTER " KALIBRUJ"
  #define TR_SETMIDPOINT       TR(CENTER "\002Cenruj dr\200\207ki/potencj",CENTER "\010Ustaw dr\200zki/potencj na \206rodku")
  #define TR_MOVESTICKSPOTS    "Poruszaj dr\200\207kami/pot"
#endif
#define TR_RXBATT              "Rx Batt:"
#define TR_TXnRX               "Tx:\0Rx:"
#define OFS_RX                 4
#define TR_ACCEL               "Acc:"
#define TR_NODATA              CENTER "BrakDAN"
#define TR_US                         "us"
#define TR_TMIXMAXMS         "TmixMaks"
#define TR_FREE_STACK     "Wolny stos"
#define TR_MENUTORESET         TR_ENTER " >> Reset"
#define TR_PPM_TRAINER         "TR"
#define TR_CH                  "KN"
#define TR_MODEL               "MODEL"
#define TR_FM                  "FL"
#define TR_MIX                 "MIX"
#define TR_EEPROMLOWMEM        "Niska pami\202\201 EEPROM"
#define TR_ALERT               "\006UWAGA"
#define TR_PRESSANYKEYTOSKIP   "Wci\206nij guzik->ignoruj"
#define TR_THROTTLENOTIDLE     "Gaz nie na zerze"
#define TR_ALARMSDISABLED      "Alarmy wy\203\200czone"
#define TR_PRESSANYKEY         TR("\010Wci\206nij jaki\206 klawisz","Wci\206nij jaki\206 klawisz")
#define TR_BADEEPROMDATA       "\006B\203\200d danych EEPROM"
#define TR_BAD_RADIO_DATA      "Bad Radio Data"
#define TR_EEPROMFORMATTING    "\004Formatowanie EEPROM"
#define TR_STORAGE_FORMAT      "Storage Preparation"
#define TR_EEPROMOVERFLOW      "Prze\203adowany EEPROM"
#define TR_MENURADIOSETUP      "USTAWIENIA RADIA"
#define TR_MENUDATEANDTIME     "DATA I CZAS"
#define TR_MENUTRAINER         "TRENER"
#define TR_MENUSPECIALFUNCS    "FUNKCJE GLOBALNE"
#define TR_MENUVERSION         "WERSJA"
#define TR_MENU_RADIO_SWITCHES            TR("PRZE\214","TEST PRZE\214")
#define TR_MENU_RADIO_ANALOGS             TR("WE-ANA","WEJ\217CIA ANALOG.")
#define TR_MENUCALIBRATION     "KALIBRACJA"
#if defined(COLORLCD)
  #define TR_TRIMS2OFFSETS     "Trymery => Subtrimy"
#else
  #define TR_TRIMS2OFFSETS     "\006Trims => Subtrims"
#endif
#define TR_CHANNELS2FAILSAFE  "Channels=>Failsafe"
#define TR_CHANNEL2FAILSAFE   "Channel=>Failsafe"
#define TR_MENUMODELSEL        TR("WYB.MODEL","WYB\216R MODELU")
#define TR_MENUSETUP           TR("USTAW","USTAW MODELU")
#define TR_MENUFLIGHTMODE     "FAZA LOTU"
#define TR_MENUFLIGHTMODES    "FAZY LOTU"
#define TR_MENUHELISETUP       "USTAW HELI"

  #define TR_MENUINPUTS        "WEJ\217CIA"
  #define TR_MENULIMITS        "WYJ\217CIA"

#define TR_MENUCURVES          "KRZYWE"
#define TR_MENUCURVE           "KRZYWA"
#define TR_MENULOGICALSWITCH    "PRZE\214. LOGICZNY"
#define TR_MENULOGICALSWITCHES   TR("PRZE\214. LOG.", "PRZE\214. LOGICZNE")
#define TR_MENUCUSTOMFUNC      "FUNKCJE SPECJALNE"
#define TR_MENUCUSTOMSCRIPTS   "W\214ASNE SKRYPTY"
#define TR_MENUTELEMETRY       "TELEMETRIA"
#define TR_MENUTEMPLATES       "SZABLONY"
#define TR_MENUSTAT            "STAT"
#define TR_MENUDEBUG           "DEBUG"
#define TR_MONITOR_CHANNELS1   "CHANNELS MONITOR 1/8"
#define TR_MONITOR_CHANNELS2   "CHANNELS MONITOR 9/16"
#define TR_MONITOR_SWITCHES    "LOGICAL SWITCHES MONITOR"
#define TR_MONITOR_CHANNELS3   "CHANNELS MONITOR 17/24"
#define TR_MONITOR_CHANNELS4   "CHANNELS MONITOR 25/32"
#define TR_MONITOR_OUTPUT_DESC "Outputs"
#define TR_MONITOR_MIXER_DESC  "Mixers"
  #define TR_RECEIVER_NUM      TR("Nr RX", "Nr odbiornika")
  #define TR_RECEIVER          INDENT "Receiver"
#define TR_MULTI_RFTUNE        TR("Freq tune", "RF Freq. fine tune")
#define TR_MULTI_RFPOWER               "RF power"
#define TR_MULTI_WBUS                  "Output"
#define TR_MULTI_TELEMETRY             "Telemetry"
#define TR_MULTI_VIDFREQ       TR("Vid. freq.", "Video frequency")
#define TR_RFPOWER       "RF Power"
#define TR_MULTI_FIXEDID               TR("FixedID", "Fixed ID")
#define TR_MULTI_OPTION        TR("Option", "Option value")
#define TR_MULTI_AUTOBIND      TR(INDENT "Bind Ch.",INDENT "Bind on channel")
#define TR_DISABLE_CH_MAP              TR("No Ch. map", "Disable Ch. map")
#define TR_DISABLE_TELEM               TR("No Telem", "Disable Telemetry")
#define TR_MULTI_DSM_AUTODTECT TR(INDENT "Autodetect", INDENT "Autodetect format")
#define TR_MULTI_LOWPOWER      TR(INDENT "Low power", INDENT "Low power mode")
#define TR_MULTI_LNA_DISABLE            INDENT "LNA disable"
#define TR_MODULE_TELEMETRY            TR(INDENT "S.Port", INDENT "S.Port link")
#define TR_MODULE_TELEM_ON             TR("ON", "Enabled")
#define TR_DISABLE_INTERNAL         TR("Disable int. RF", "Disable internal RF")
#define TR_MODULE_NO_SERIAL_MODE       TR("!serial mode", "Not in serial mode")
#define TR_MODULE_NO_INPUT             TR("No input", "No serial input")
#define TR_MODULE_NO_TELEMETRY         TR3( "No telmetry", "No MULTI_TELEMETRY", "No telemetry (enable MULTI_TELEMETRY)")
#define TR_MODULE_WAITFORBIND          "Bind to load protocol"
#define TR_MODULE_BINDING              "Binding"
#define TR_MODULE_UPGRADE_ALERT        TR3("Upg. needed", "Module upgrade required", "Module\036Upgrade required")
#define TR_MODULE_UPGRADE              TR("Upg. advised", "Module update recommended")
#define TR_PULSE_RATE                  "Pulse rate"
#define TR_LINK_SPEED                  "Link speed"
#define TR_REBIND                      "Rebinding required"
#define TR_REG_OK                      "Registration ok"
#define TR_BIND_OK                     "Bind successful"
#define TR_BINDING_CH1_8_TELEM_ON               "Ch1-8 Telem ON"
#define TR_BINDING_CH1_8_TELEM_OFF               "Ch1-8 Telem OFF"
#define TR_BINDING_CH9_16_TELEM_ON               "Ch9-16 Telem ON"
#define TR_BINDING_CH9_16_TELEM_OFF               "Ch9-16 Telem OFF"
#define TR_PROTOCOL_INVALID            TR("Prot. invalid", "Protocol invalid")
#define TR_MODULE_STATUS                TR(INDENT "Status", INDENT "Module Status")
#define TR_MODULE_SYNC                 TR(INDENT "Sync", INDENT "Proto Sync Status")
#define TR_MULTI_SERVOFREQ     TR("Servo rate", "Servo update rate")
#define TR_MULTI_MAX_THROW             TR("Max. Throw", "Enable max. throw")
#define TR_MULTI_RFCHAN                TR("RF Channel", "Select RF channel")
#define TR_SYNCMENU            "[Synch]"
#define TR_LIMIT               INDENT "Limit"
#define TR_MINRSSI             "Min Rssi"
#define TR_LATITUDE            "Szer. geo."
#define TR_LONGITUDE           "D\203.   geo."
#define TR_GPSCOORD            TR("Koord.GPS","Typ koordynat GPS")
#define TR_VARIO               TR("Wario","Wariometr")
#define TR_PITCH_AT_ZERO       INDENT "Ton przy Zerze"
#define TR_PITCH_AT_MAX        INDENT "Ton przy Max"
#define TR_REPEAT_AT_ZERO      INDENT "Powt\205rz przy Zerze"
#define TR_SHUTDOWN            "Wy\203\200czanie.."
#define TR_SAVEMODEL           "Zapisz ustwienia modelu"
#define TR_BATT_CALIB          "Kalib:Baterii"
#define TR_CURRENT_CALIB       " +=\006Kalibracja pr\200du"
#define TR_VOLTAGE             TR (INDENT"Napi\202cie",INDENT"\221r\205d\203o Napi\202cia")
#define TR_CURRENT             TR (INDENT"Pr\200d",INDENT"\221r\205d\203o Pr\200du")
#define TR_SELECT_MODEL        "Wyb\205r modelu"
#define TR_SELECT_MODE                 "Select mode"
#define TR_CREATE_CATEGORY     "Create Category"
#define TR_RENAME_CATEGORY     "Rename Category"
#define TR_DELETE_CATEGORY     "Delete Category"
#define TR_CREATE_MODEL        "Nowy model"
#define TR_DUPLICATE_MODEL     "Duplicate Model"
#define TR_COPY_MODEL          "Skopiuj model"
#define TR_MOVE_MODEL          "Przenie\206 model"
#define TR_BACKUP_MODEL        "Zbackupuj model"
#define TR_DELETE_MODEL        "Skasuj model"
#define TR_RESTORE_MODEL       "Odtw\205rz model"
#define TR_DELETE_ERROR        "Delete error"
#define TR_CAT_NOT_EMPTY       "Category is not empty"
#define TR_SDCARD_ERROR        "B\203\200d karty SD"
#define TR_NO_SDCARD           "Brak karty SD"
#define TR_WAITING_FOR_RX              "Waiting for RX..."
#define TR_WAITING_FOR_TX              "Waiting for TX..."
#define TR_WAITING_FOR_MODULE          TR("Waiting module", "Waiting for module...")
#define TR_NO_TOOLS                    "No tools available"
#define TR_NORMAL                      "Normal"
#define TR_NOT_INVERTED                "Not inv"
#define TR_NOT_CONNECTED               "!Connected"
#define TR_CONNECTED                   "Connected"
#define TR_FLEX_915                    "Flex 915MHz"
#define TR_FLEX_868                    "Flex 868MHz"
#define TR_16CH_WITHOUT_TELEMETRY      TR("16CH without telem.", "16CH without telemetry")
#define TR_16CH_WITH_TELEMETRY         TR("16CH with telem.", "16CH with telemetry")
#define TR_8CH_WITH_TELEMETRY          TR("8CH with telem.", "8CH with telemetry")
#define TR_EXT_ANTENNA                 "Ext. antenna"
#define TR_PIN                         "Pin"
#define TR_UPDATE_RX_OPTIONS           "Update RX options?"
#define TR_UPDATE_TX_OPTIONS           "Update TX options?"
#define TR_MODULES_RX_VERSION          "Modules / RX version"
#define TR_MENU_MODULES_RX_VERSION     "MODULES / RX VERSION"
#define TR_MENU_FIRM_OPTIONS           "FIRMWARE OPTIONS"
#define TR_GYRO                        "Gyro"
#define TR_STICKS_POTS_SLIDERS         "Sticks/Pots/Sliders"
#define TR_PWM_STICKS_POTS_SLIDERS     "PWM Sticks/Pots/Sliders"
#define TR_RF_PROTOCOL                 "RF Protocol"
#define TR_MODULE_OPTIONS              "Module options"
#define TR_POWER                       "Power"
#define TR_NO_TX_OPTIONS               "No TX options"
#define TR_RTC_BATT                    "RTC Batt"
#define TR_POWER_METER_EXT             "Power Meter (EXT)"
#define TR_POWER_METER_INT             "Power Meter (INT)"
#define TR_SPECTRUM_ANALYSER_EXT       "Spectrum (EXT)"
#define TR_SPECTRUM_ANALYSER_INT       "Spectrum (INT)"
#define TR_SDCARD_FULL                 "Karta Pe\203na "
#define TR_NEEDS_FILE                  "NEEDS FILE"
#define TR_EXT_MULTI_SPEC              "opentx-inv"
#define TR_INT_MULTI_SPEC              "stm-opentx-noinv"
#define TR_INCOMPATIBLE        "Niekompatybilne"
#define TR_WARNING             "UWAGA"
#define TR_EEPROMWARN          "EEPROM"
#define TR_STORAGE_WARNING     "STORAGE"
#define TR_EEPROM_CONVERTING   "Konwersja EEPROM"
#define TR_THROTTLEWARN        "GAZ"
#define TR_ALARMSWARN          "ALARM"
#define TR_SWITCHWARN          TR("Prze\203\200","Kontrola")
#define TR_FAILSAFEWARN        "FAILSAFE"
#define TR_TEST_WARNING        TR("TESTING", "TEST BUILD")
#define TR_TEST_NOTSAFE        "Use for tests only"
#define TR_WRONG_SDCARDVERSION TR("Expected ver: ","Expected version: ")
#define TR_WARN_RTC_BATTERY_LOW        "RTC Battery low"
#define TR_WARN_MULTI_LOWPOWER         "Low power mode"
#define TR_BATTERY                     "BATTERY"
#define TR_WRONG_PCBREV        "Wrong PCB detected"
#define TR_EMERGENCY_MODE      "EMERGENCY MODE"
#define TR_PCBREV_ERROR        "PCB error"
#define TR_NO_FAILSAFE         "Brak Failsafe"
#define TR_KEYSTUCK            "Blokada klucza"
#define TR_INVERT_THR          TR("Odw.Gaz?","Odwr\205\201 gaz?")
#define TR_SPEAKER_VOLUME      INDENT "G\203o\206no\206\201"
#define TR_LCD                 "LCD"
#define TR_BRIGHTNESS          INDENT "Jasno\206\201"
#define TR_CPU_TEMP            "Temp. CPU\016>"
#define TR_CPU_CURRENT         "Pr\200d"
#define TR_CPU_MAH             "Zu\207ycie"
#define TR_COPROC              "CoProc."
#define TR_COPROC_TEMP         "Temp. MB"
#define TR_CAPAWARNING         INDENT "Ma\203a pojemno\206\201"
#define TR_TEMPWARNING         INDENT "Przegrzanie"
#define TR_FUNC                "Funkc"
#define TR_V1                  "V1"
#define TR_V2                  "V2"
#define TR_DURATION            "Czas trwania"
#define TR_DELAY               "Op\205\210nienie"
#define TR_SD_CARD             "SD"
#define TR_SDHC_CARD           "SD-HC"
#define TR_NO_SOUNDS_ON_SD     "Brak d\210wi\202k\205w na SD"
#define TR_NO_MODELS_ON_SD     "Brak modelu na SD"
#define TR_NO_BITMAPS_ON_SD    "Brak obrazk\205w na SD"
#define TR_NO_SCRIPTS_ON_SD    "Brak skrypt\205w na SD"
#define TR_SCRIPT_SYNTAX_ERROR TR("Syntax error", "Skrypt:syntax error")
#define TR_SCRIPT_PANIC        "Skrypt:panic"
#define TR_SCRIPT_KILLED       "Skrypt:killed"
#define TR_SCRIPT_ERROR        "Nieznany b\203\200d"
#define TR_PLAY_FILE           "Odtw\205rz"
#define TR_DELETE_FILE         "Skasuj"
#define TR_COPY_FILE           "Kopiuj"
#define TR_RENAME_FILE         "Zmie\204 nazw\202"
#define TR_ASSIGN_BITMAP       "Wybierz obrazek"
#define TR_ASSIGN_SPLASH       "Splash screen"
#define TR_EXECUTE_FILE        "Wykonaj"
#define TR_REMOVED             " skasowane"
#define TR_SD_INFO             "Informacje"
#define TR_SD_FORMAT           "Formatuj"
#define TR_NA                  "[X]"
#define TR_HARDWARE            "HARDWARE"
#define TR_FORMATTING          "Formatowanie.."
#define TR_TEMP_CALIB          "Temp. kalibracji"
#define TR_TIME                "Czas"
#define TR_MAXBAUDRATE         "Max bauds"

#define TR_BLUETOOTH            "Bluetooth"
#define TR_BLUETOOTH_DISC       "Discover"
#define TR_BLUETOOTH_INIT       "Init"
#define TR_BLUETOOTH_DIST_ADDR  "Dist addr"
#define TR_BLUETOOTH_LOCAL_ADDR "Local addr"
#define TR_BLUETOOTH_PIN_CODE   "PIN code"
#define TR_BAUDRATE             "Pr\202dko\206\201 BT"
#define LEN_BLUETOOTH_MODES     "\011"
#if defined(PCBX9E)
#define TR_BLUETOOTH_MODES      "---\0     ""Enabled\0 "
#else
#define TR_BLUETOOTH_MODES      "---\0     ""Telemetry""Trainer\0"
#endif
#define TR_SD_INFO_TITLE       "SD INFO"
#define TR_SD_TYPE             "Typ:"
#define TR_SD_SPEED            "Pr\202dko\206\201:"
#define TR_SD_SECTORS          "Sektory :"
#define TR_SD_SIZE             "Wielko\206\201:"
#define TR_TYPE                INDENT TR_SD_TYPE
#define TR_GLOBAL_VARS         "Zmienne Globalne"
#define TR_GVARS               "Zmien.Glob."
#define TR_GLOBAL_VAR          "Zmienna Globalna"
#define TR_MENUGLOBALVARS      "ZMIENNE GLOBALNE"
#define TR_OWN                 "W\203asno\206\201"
#define TR_DATE                "Data"
#define TR_MONTHS              { "Jan", "Fev", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" }
#define TR_ROTARY_ENCODER      "R.E."
#define TR_INVERT_ROTARY       "Invert Rotary"
#define TR_CHANNELS_MONITOR    "MONITOR KANA\214\216W"
#define TR_MIXERS_MONITOR      "MONITOR MIKSER"
#define TR_PATH_TOO_LONG       "\217cie\207ka za d\203uga"
#define TR_VIEW_TEXT           "Poka\207 tekst"
#define TR_FLASH_BOOTLOADER    "Flash bootloader"
#define TR_FLASH_EXTERNAL_DEVICE "Sflashuj Modu\203 Zewn\202trzny"
#define TR_FLASH_RECEIVER_OTA          "Flash receiver OTA"
#define TR_FLASH_RECEIVER_BY_EXTERNAL_MODULE_OTA "Flash RX by ext. OTA"
#define TR_FLASH_RECEIVER_BY_INTERNAL_MODULE_OTA "Flash RX by int. OTA"
#define TR_FLASH_FLIGHT_CONTROLLER_BY_EXTERNAL_MODULE_OTA "Flash FC by ext. OTA"
#define TR_FLASH_FLIGHT_CONTROLLER_BY_INTERNAL_MODULE_OTA "Flash FC by int. OTA"
#define TR_FLASH_BLUETOOTH_MODULE      TR("Flash BT module", "Flash Bluetooth module")
#define TR_FLASH_POWER_MANAGEMENT_UNIT          "Flash pwr mngt unit"
#define TR_CURRENT_VERSION             TR("Current vers. ", "Current version: ")
#define TR_HW_REV                      "HW Rev"
#define TR_FLASH_INTERNAL_MODULE "Sflashuj Modu\203 Wewn\202trzny"
#define TR_FLASH_INTERNAL_MULTI        TR("Flash Int. Multi", "Flash Internal Multi")
#define TR_FLASH_EXTERNAL_MODULE       "Flash external module"
#define TR_FLASH_EXTERNAL_MULTI        TR("Flash Ext. Multi", "Flash External Multi")
#define TR_FLASH_EXTERNAL_ELRS         TR("Flash Ext. ELRS", "Flash External ELRS")
#define TR_FIRMWARE_UPDATE_ERROR TR("FW update Error","Firmware update error")
#define TR_FIRMWARE_UPDATE_SUCCESS     "Flash successful"
#define TR_WRITING                     "Zapis...  "
#define TR_CONFIRM_FORMAT              "Zatwierd\210 Format?"
#define TR_INTERNALRF                  "Wewn.Modu\203 RF"
#define TR_INTERNAL_MODULE             TR("Int. module","Internal module")
#define TR_EXTERNAL_MODULE             TR("Ext. module","External module")
#define TR_OPENTX_UPGRADE_REQUIRED     "OpenTX upgrade required"
#define TR_TELEMETRY_DISABLED          "Telem. disabled"
#define TR_MORE_OPTIONS_AVAILABLE      "More options available"
#define TR_NO_MODULE_INFORMATION       "No module information"
#define TR_EXTERNALRF          "Zewn.Modu\203 RF"
#define TR_FAILSAFE            TR(INDENT"Failsafe",INDENT"Tryb Failsafe")
#define TR_FAILSAFESET         "USTAWIENIE FAILSAFE"
#define TR_REG_ID                      "Reg. ID"
#define TR_OWNER_ID                    "Owner ID"
#define TR_PINMAPSET                   "PINMAP"
#define TR_HOLD                "Hold"
#define TR_HOLD_UPPERCASE              "HOLD"
#define TR_NONE                "None"
#define TR_NONE_UPPERCASE              "NONE"
#define TR_MENUSENSOR          "CZUJNIK"
#define TR_POWERMETER_PEAK             "Peak"
#define TR_POWERMETER_POWER            "Power"
#define TR_POWERMETER_ATTN             "Attn"
#define TR_POWERMETER_FREQ             "Freq."
#define TR_MENUTOOLS                   "TOOLS"
#define TR_TURN_OFF_RECEIVER           "Turn off receiver"
#define TR_STOPPING                    "Stopping..."
#define TR_MENU_SPECTRUM_ANALYSER      "SPECTRUM ANALYSER"
#define TR_MENU_POWER_METER            "POWER METER"
#define TR_SENSOR              "SENSOR"
#define TR_COUNTRYCODE         "Kod regionu"
#define TR_USBMODE             "USB Mode"
#define TR_JACKMODE                    "Jack Mode"
#define TR_VOICELANG           "J\202zyk g\203osu"
#define TR_UNITSSYSTEM         "Jednostki"
#define TR_EDIT                "Edytuj"
#define TR_INSERT_BEFORE       "Wstaw przed"
#define TR_INSERT_AFTER        "Wstaw za"
#define TR_COPY                "Kopiuj"
#define TR_MOVE                "Przenie\206"
#define TR_PASTE               "Wklej"
#define TR_DELETE              "Kasuj"
#define TR_INSERT              "Wstaw"
#define TR_RESET_FLIGHT        "Zeruj lot"
#define TR_RESET_TIMER1        "Zeruj Timer1"
#define TR_RESET_TIMER2        "Zeruj Timer2"
#define TR_RESET_TIMER3        "Zeruj Timer3"
#define TR_RESET_TELEMETRY     "Wyczy\206\201 telemetri\202"
#define TR_STATISTICS          "Statystyki"
#define TR_ABOUT_US            "O nas"
#define TR_USB_JOYSTICK        "USB Joystick (HID)"
#define TR_USB_MASS_STORAGE    "USB Storage (SD)"
#define TR_USB_SERIAL          "USB Serial (Debug)"
#define TR_USB_TELEMETRY       "USB Telem mirror"
#define TR_SETUP_SCREENS       "Setup screens"
#define TR_MONITOR_SCREENS     "Monitors"
#define TR_AND_SWITCH          "Prze\203\200cznik AND"
#define TR_SF                  "FS"
#define TR_GF                  "FG"
#define TR_SPEAKER             INDENT"G\203o\206nik"
#define TR_BUZZER              INDENT"Brz\202czyk"
#define TR_BYTES               "bajty"
#define TR_MODULE_BIND         TR("[BND]","[Bind]")
#define TR_POWERMETER_ATTN_NEEDED      "Attenuator needed"
#define TR_PXX2_SELECT_RX              "Select RX"
#define TR_PXX2_DEFAULT                "<default>"
#define TR_BT_SELECT_DEVICE            "Select device"
#define TR_DISCOVER             "Discover"
#define TR_BUTTON_INIT                 BUTTON("Init")
#define TR_WAITING                     "Waiting..."
#define TR_RECEIVER_DELETE             "Delete receiver?"
#define TR_RECEIVER_RESET              "Reset receiver?"
#define TR_SHARE                       "Share"
#define TR_BIND                        "Bind"
#define TR_REGISTER                    TR("Reg", "Register")
#define TR_MODULE_RANGE        TR("[Zas]","Zasi\202g")
#define TR_RECEIVER_OPTIONS            TR("REC. OPTIONS", "RECEIVER OPTIONS")
#define TR_DEL_BUTTON                  BUTTON(TR("Del", "Delete"))
#define TR_RESET_BTN           "[Reset]"
#define TR_DEBUG                       "Debug"
#define TR_KEYS_BTN                BUTTON(TR("SW","Switches"))
#define TR_ANALOGS_BTN                 BUTTON(TR("Analog","Analogs"))
#define TR_TOUCH_NOTFOUND              "Touch hardware not found"
#define TR_TOUCH_EXIT                  "Touch screen to exit"
#define TR_CALIBRATION                   "Calibration"
#define TR_SET                 "[Set]"
#define TR_TRAINER             "Port Trenera"
#define TR_CHANS                       "Chans"
#define TR_ANTENNAPROBLEM      CENTER "Problem z anten\200 TX"
#define TR_MODELIDUSED         TR("ID zaj\202te","ID modelu zaj\202te")
#define TR_MODULE              "Modu\203 "
#define TR_RX_NAME                     "Rx Name"
#define TR_TELEMETRY_TYPE      TR("Typ ", "Typ Telemetrii")
#define TR_TELEMETRY_SENSORS   "Czujniki"
#define TR_VALUE               "Warto\206\201"
#define TR_TOPLCDTIMER         "Top LCD Timer"
#define TR_UNIT                "Jedn"
#define TR_TELEMETRY_NEWSENSOR INDENT "Dodaj nowy czujnik..."
#define TR_CHANNELRANGE        TR(INDENT "ZakrKn",INDENT "Zakres kana\203u")
#define TR_AFHDS3_RX_FREQ              TR("RX freq.", "RX frequency")
#define TR_AFHDS3_ONE_TO_ONE_TELEMETRY TR("Unicast/Tel.", "Unicast/Telemetry")
#define TR_AFHDS3_ONE_TO_MANY          "Multicast"
#define TR_AFHDS3_ACTUAL_POWER         TR("Act. pow", "Actual power")
#define TR_AFHDS3_POWER_SOURCE         TR("Power src.", "Power source")
#define TR_ANTENNACONFIRM1     "EXT. ANTENNA"
#if defined(PCBX12S)
#define LEN_ANTENNA_MODES      "\023"
#define TR_ANTENNA_MODES       "Internal\0          ""Ask\0               ""Per model\0         ""Internal + External"
#else
#define LEN_ANTENNA_MODES      "\011"
#define TR_ANTENNA_MODES       "Internal\0""Ask\0     ""Per model""External"
#endif
#define TR_USE_INTERNAL_ANTENNA        TR("Use int. antenna", "Use internal antenna")
#define TR_USE_EXTERNAL_ANTENNA        TR("Use ext. antenna", "Use external antenna")
#define TR_ANTENNACONFIRM2     TR("Check antenna", "Make sure antenna is installed!")
#define TR_MODULE_PROTOCOL_FLEX_WARN_LINE1                "Requires non"
#define TR_MODULE_PROTOCOL_FCC_WARN_LINE1         "Requires FCC"
#define TR_MODULE_PROTOCOL_EU_WARN_LINE1          "Requires EU"
#define TR_MODULE_PROTOCOL_WARN_LINE2                "certified firmware"
#define TR_LOWALARM            INDENT "Alarm niski"
#define TR_CRITICALALARM       INDENT "Alarm krytyczny"
#define TR_RSSIALARM_WARN             TR("RSSI","TELEMETRY RSSI")
#define TR_NO_RSSIALARM                TR(INDENT "Alarms disabled", INDENT "Telemetry alarms disabled")
#define TR_DISABLE_ALARM               TR(INDENT "Disable alarms", INDENT "Disable telemetry alarms")
#define TR_ENABLE_POPUP        "Aktywuj Popup"
#define TR_DISABLE_POPUP       "Wy\203\200cz Popup"
#define TR_POPUP               "Popup"
#define TR_MIN                 "Min"
#define TR_MAX                 "Max"
#define TR_CURVE_PRESET        "Predef..."
#define TR_PRESET              "Predef."
#define TR_MIRROR              "Lustro"
#define TR_CLEAR               "Wyczy\206\201"
#define TR_RESET               "Resetuj"
#define TR_RESET_SUBMENU       "Reset..."
#define TR_COUNT               "Punkty"
#define TR_PT                  "pkt"
#define TR_PTS                 "pkty"
#define TR_SMOOTH              "G\203adka"
#define TR_COPY_STICKS_TO_OFS  TR("Cpy stick->subtrim", "Skopiuj Dr\200\207ki Do Offsetu")
#define TR_COPY_MIN_MAX_TO_OUTPUTS     TR("Cpy min/max to all",  "Copy min/max/center to all outputs")
#define TR_COPY_TRIMS_TO_OFS   TR("Cpy trim->subtrim", "Skopiuj Trymery Do subtrymer\205w")
#define TR_INCDEC              "Zwi\202ksz/Zmnie"
#define TR_GLOBALVAR           "Zm.Global."
#define TR_MIXSOURCE           "\221r\205d\203o Miks."
#define TR_CONSTANT            "Stala   "
#define TR_PERSISTENT_MAH      TR(INDENT "Zap. mAh", INDENT "Zapisz mAh")
#define TR_PREFLIGHT           "Lista Ostrze\207e\204"
#define TR_CHECKLIST           TR(INDENT "Czeklista",INDENT "Poka\207 List\202 Ostrze\207e\204")
#define TR_FAS_OFFSET          TR(INDENT "FAS Ofs", INDENT "FAS Offset")
#define TR_AUX_SERIAL_MODE     "Port szreg."
#define TR_AUX2_SERIAL_MODE    "Port szreg. 2"
#define TR_SCRIPT              "Skrypt"
#define TR_INPUTS              "Wej\206cia"
#define TR_OUTPUTS             "Wyj\206cia"
#define STR_EEBACKUP            "Backup EEPROMu"
#define STR_FACTORYRESET        "Ustaw.Fabrycz"
#define TR_CONFIRMRESET        "WYkasowa\201 wszytkie modele?    "
#define TR_TOO_MANY_LUA_SCRIPTS "Za du\207o skrypt\205w Lua!"
#define TR_SPORT_UPDATE_POWER_MODE     "SP Power"
#define LEN_SPORT_UPDATE_POWER_MODES   "\004"
#define TR_SPORT_UPDATE_POWER_MODES    "AUTO""ON\0 "
#define TR_NO_TELEMETRY_SCREENS        "No Telemetry Screens"
#define TR_TOUCH_PANEL                 "Touch panel:"

// Horus and Taranis column headers
#define TR_PHASES_HEADERS_NAME         "Nazwa"
#define TR_PHASES_HEADERS_SW           "Prze\203\200"
#define TR_PHASES_HEADERS_RUD_TRIM     "Trymer SK"
#define TR_PHASES_HEADERS_ELE_TRIM     "Trymer SW"
#define TR_PHASES_HEADERS_THT_TRIM     "Trymer Gazu"
#define TR_PHASES_HEADERS_AIL_TRIM     "Trymer Lotek"
#define TR_PHASES_HEADERS_CH5_TRIM     "Trymer 5"
#define TR_PHASES_HEADERS_CH6_TRIM     "Trymer 6"
#define TR_PHASES_HEADERS_FAD_IN       "Pojawian"
#define TR_PHASES_HEADERS_FAD_OUT      "Zanik"

#define TR_LIMITS_HEADERS_NAME         "Nazwa"
#define TR_LIMITS_HEADERS_SUBTRIM      "Subtrim"
#define TR_LIMITS_HEADERS_MIN          "Min"
#define TR_LIMITS_HEADERS_MAX          "Max"
#define TR_LIMITS_HEADERS_DIRECTION    "Kierunek"
#define TR_LIMITS_HEADERS_CURVE        "Krzywa"
#define TR_LIMITS_HEADERS_PPMCENTER    "\217rodek PPM"
#define TR_LIMITS_HEADERS_SUBTRIMMODE  "Tryb subtrim"

#define TR_LSW_HEADERS_FUNCTION        "Funkcja"
#define TR_LSW_HEADERS_V1              "V1"
#define TR_LSW_HEADERS_V2              "V2"
#define TR_LSW_HEADERS_ANDSW           "AND Prze\203\200"
#define TR_LSW_HEADERS_DURATION        "CzasTrwa"
#define TR_LSW_HEADERS_DELAY           "Op\205\210n"

#define TR_GVAR_HEADERS_NAME          "Name"
#define TR_GVAR_HEADERS_FM0           "Value on FM0"
#define TR_GVAR_HEADERS_FM1           "Value on FM1"
#define TR_GVAR_HEADERS_FM2           "Value on FM2"
#define TR_GVAR_HEADERS_FM3           "Value on FM3"
#define TR_GVAR_HEADERS_FM4           "Value on FM4"
#define TR_GVAR_HEADERS_FM5           "Value on FM5"
#define TR_GVAR_HEADERS_FM6           "Value on FM6"
#define TR_GVAR_HEADERS_FM7           "Value on FM7"
#define TR_GVAR_HEADERS_FM8           "Value on FM8"

// Horus footer descriptions
#define TR_LSW_DESCRIPTIONS    { "Comparison type or function", "First variable", "Second variable or constant", "Second variable or constant", "Additional condition for line to be enabled", "Minimum ON duration of the logical switch", "Minimum TRUE duration for the switch to become ON" }

// Horus layouts and widgets
#define TR_FIRST_CHANNEL               "First channel"
#define TR_FILL_BACKGROUND             "Fill background?"
#define TR_BG_COLOR                    "BG Color"
#define TR_SLIDERS_TRIMS               "Sliders+Trims"
#define TR_SLIDERS                     "Sliders"
#define TR_FLIGHT_MODE                 "Flight mode"
#define TR_INVALID_FILE                "Invalid File"
#define TR_TIMER_SOURCE                "Timer source"
#define TR_SIZE                        "Size"
#define TR_SHADOW                      "Shadow"
#define TR_TEXT                        "Text"
#define TR_COLOR                       "Color"
#define TR_MAIN_VIEW_X                 "Main view X"
#define TR_PANEL1_BACKGROUND           "Panel1 background"
#define TR_PANEL2_BACKGROUND           "Panel2 background"

// About screen
#define TR_ABOUTUS             TR("   O   ", "  O  ")

#define TR_ABOUT_OPENTX_1      TR("OpenTX\001is\001open\001source,\001non", "OpenTX is open source, non-")
#define TR_ABOUT_OPENTX_2      TR("commercial,\001wo\001warranties.", "commercial and comes with no")
#define TR_ABOUT_OPENTX_3      TR("It\001was\001developed\001for\001free.", "warranties. It was developed")
#define TR_ABOUT_OPENTX_4      TR("Support through donations", "for free. Support through")
#define TR_ABOUT_OPENTX_5      TR("is welcome!", "donations is welcome!")

#define TR_ABOUT_BERTRAND_1    "Bertrand Songis"
#define TR_ABOUT_BERTRAND_2    "OpenTX g\203\205wny autor"
#define TR_ABOUT_BERTRAND_3    "Companion wsp\205\203-autor"

#define TR_ABOUT_MIKE_1        "Mike Blandford"
#define TR_ABOUT_MIKE_2        "Guru kodu i driwer\205w"
#define TR_ABOUT_MIKE_3        TR("Arguably,\001one\001of\001the\001best", "Arguably, one of the best")
#define TR_ABOUT_MIKE_4        "Inspirational"

#define TR_ABOUT_ROMOLO_1      "Romolo Manfredini"
#define TR_ABOUT_ROMOLO_2      "Companion wsp\205\203-autor"
#define TR_ABOUT_ROMOLO_3      ""

#define TR_ABOUT_ANDRE_1       "Andre Bernet"
#define TR_ABOUT_ANDRE_2       "Funkcjonalno\206\201, u\207ytkowo\206\201,"
#define TR_ABOUT_ANDRE_3       "debugging, documentacja"

#define TR_ABOUT_ROB_1         "Rob Thomson"
#define TR_ABOUT_ROB_2         "openrcforums webmaster"

#define TR_ABOUT_KJELL_1       "Kjell Kernen"
#define TR_ABOUT_KJELL_2       "www.open-tx.org main author"
#define TR_ABOUT_KJELL_3       "OpenTX Recorder author"
#define TR_ABOUT_KJELL_4       "Companion contributor"

#define TR_ABOUT_MARTIN_1      "Martin Hotar"
#define TR_ABOUT_MARTIN_2      "Projekty graficzne"

#if defined(MANUFACTURER_FRSKY)
  #define TR_ABOUT_HARDWARE_1         "FrSky"
  #define TR_ABOUT_HARDWARE_2         "Proucent/projektant Hardware"
  #define TR_ABOUT_HARDWARE_3         "Wsp\205\203autor firmware"
#elif defined(MANUFACTURER_RADIOMASTER)
  #define TR_ABOUT_HARDWARE_1         "Radiomaster"
  #define TR_ABOUT_HARDWARE_2         "Proucent/projektant Hardware"
  #define TR_ABOUT_HARDWARE_3         "Wsp\205\203autor firmware"
#elif defined(MANUFACTURER_JUMPER)
  #define TR_ABOUT_HARDWARE_1         "JumperRC"
  #define TR_ABOUT_HARDWARE_2         "Proucent/projektant Hardware"
  #define TR_ABOUT_HARDWARE_3         "Wsp\205\203autor firmware"
#else
  #define TR_ABOUT_HARDWARE_1         "Brent Nelson"
  #define TR_ABOUT_HARDWARE_2         "Sky9x designer/výrobce"
  #define TR_ABOUT_HARDWARE_3         ""
#endif

#define TR_ABOUT_PARENTS_1     "Projekty macierzyste"
#define TR_ABOUT_PARENTS_2     TR("Ersky9x (Mike Blandford)", "Ersky9x (Mike Blandford)")
#define TR_ABOUT_PARENTS_3     "ER9X (Erez Raviv)"
#define TR_ABOUT_PARENTS_4     "TH9X (Thomas Husterer)"

#define TR_CHR_SHORT           'k'
#define TR_CHR_LONG            'd'
#define TR_CHR_TOGGLE          't'
#define TR_CHR_HOUR            'h'
#define TR_CHR_INPUT           'I'   // Values between A-I will work

#define TR_BEEP_VOLUME         "G\203o\206n pikan"
#define TR_WAV_VOLUME          "G\203o\206n wav  "
#define TR_BG_VOLUME           "G\203o\206n t\203a"

#define TR_TOP_BAR             "G\205rnPas"
#define TR_FLASH_ERASE                 "Flash erase..."
#define TR_FLASH_WRITE                 "Flash write..."
#define TR_OTA_UPDATE                  "OTA update..."
#define TR_MODULE_RESET                "Module reset..."
#define TR_UNKNOWN_RX                  "Unknown RX"
#define TR_UNSUPPORTED_RX              "Unsupported RX"
#define TR_OTA_UPDATE_ERROR            "OTA update error"
#define TR_DEVICE_RESET                "Device reset..."
#define TR_ALTITUDE            INDENT "Wysoko\206\201"
#define TR_SCALE               "Skala"
#define TR_VIEW_CHANNELS       "Poka\207 Kana\203y"
#define TR_VIEW_NOTES          "Poka\207 Notki"
#define TR_MODEL_SELECT        "Model Select"
#define TR_MODS_FORBIDDEN      "Modyfikacje zabronione!"
#define TR_UNLOCKED            "Odblokowane"
#define TR_ID                  "ID"
#define TR_PRECISION           "Precyzja "
#define TR_RATIO               "Wsp\205\203cz."
#define TR_FORMULA             "Formu\203a"
#define TR_CELLINDEX           "Cell index"
#define TR_LOGS                "Logi"
#define TR_OPTIONS             "Opcje  "
#define TR_FIRMWARE_OPTIONS    "Firmware options"

#define TR_ALTSENSOR           "Alt sensor"
#define TR_CELLSENSOR          "Cell sensor"
#define TR_GPSSENSOR           "GPS sensor"
#define TR_CURRENTSENSOR       "Sensor"
#define TR_AUTOOFFSET          "Auto Ofset"
#define TR_ONLYPOSITIVE        "Dodatni"
#define TR_FILTER              "Filtr"
#define TR_TELEMETRYFULL       "Wszyskie miejsca zaj\202te!"
#define TR_SERVOS_OK           "Servos OK"
#define TR_SERVOS_KO           "Servos KO"
//TODO: translation
#define TR_INVERTED_SERIAL     INDENT "Invert"
#define TR_IGNORE_INSTANCE     INDENT "Ignoruj przypadek"
#define TR_DISCOVER_SENSORS    "Znajd\210 nowe czujniki"
#define TR_STOP_DISCOVER_SENSORS "Szukanie STOP "
#define TR_DELETE_ALL_SENSORS  "Usu\204 czujniki     "
#define TR_CONFIRMDELETE       "Usun\200\201 wszystkie?"
#define TR_SELECT_WIDGET       "Select widget"
#define TR_REMOVE_WIDGET       "Remove widget"
#define TR_WIDGET_SETTINGS     "Widget settings"
#define TR_REMOVE_SCREEN       "Remove screen"
#define TR_SETUP_WIDGETS       "Setup widgets"
#define TR_USER_INTERFACE      "User interface"
#define TR_THEME               "Theme"
#define TR_SETUP               "Setup"
#define TR_MAINVIEWX           "Main view X"
#define TR_LAYOUT              "Layout"
#define TR_ADDMAINVIEW         "Add main view"
#define TR_BACKGROUND_COLOR    "Background color"
#define TR_MAIN_COLOR          "Main color"
#define TR_BAR2_COLOR                  "Secondary bar color"
#define TR_BAR1_COLOR                  "Main bar color"
#define TR_TEXT_COLOR                  "Text color"
#define TR_TEXT_VIEWER                 "Text Viewer"

#define TR_MENU_INPUTS         "\314Wej\206ci"
#define TR_MENU_LUA            "\322SkryptyLUA"
#define TR_MENU_STICKS         "\307Dr\200\207ki"
#define TR_MENU_POTS           "\310Pots"
#define TR_MENU_MAX            "\315MAX"
#define TR_MENU_HELI           "\316Cyclic"
#define TR_MENU_TRIMS          "\313Trymy"
#define TR_MENU_SWITCHES       "\312Prze\203czn"
#define TR_MENU_LOGICAL_SWITCHES "\312Prze\203\200czn.Logicz"
#define TR_MENU_TRAINER        "\317Trener "
#define TR_MENU_CHANNELS       "\320Kana\203y  "
#define TR_MENU_GVARS          "\311ZmGlo"
#define TR_MENU_TELEMETRY      "\321Telemetry"
#define TR_MENU_DISPLAY        "DISPLAY"
#define TR_MENU_OTHER          "Inny "
#define TR_MENU_INVERT         "Odwr\205\201"
#define TR_JITTER_FILTER       "ADC Filter"
#define TR_RTC_CHECK           TR("Check RTC", "Check RTC voltage")
#define TR_AUTH_FAILURE                "Auth-failure"
#define TR_RACING_MODE                 "Racing mode"

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

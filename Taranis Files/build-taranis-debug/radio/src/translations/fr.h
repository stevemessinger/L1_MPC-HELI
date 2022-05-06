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

// FR translations author: Andre Bernet <bernet.andre@gmail.com>

/*
 * !!!!! DO NOT EDIT fr.h - EDIT fr.h.txt INSTEAD !!!!!!!
 *
 * In order to make translations easier fr.h.txt is parsed and national
 * characters are replaced by bitmap codes. The result is fr.h.
 *
 * See translate.py in the util folder for the list of character codes
 *
 * !!!!! DO NOT EDIT fr.h - EDIT fr.h.txt INSTEAD !!!!!!!
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
#define LEN_OFFON                      "\003"
#define TR_OFFON                       "OFF""ON\0"

#define LEN_MMMINV                     "\003"
#define TR_MMMINV                      "---""INV"

#define LEN_VBEEPMODE                  "\005"
#define TR_VBEEPMODE                   "Aucun""Alarm""NoKey""Tout\0"

#define LEN_VBLMODE                    TR("\004", "\011")
#define TR_VBLMODE                     TR("OFF\0""Btns""Ctrl""Tous""ON\0","OFF\0     ""Touches\0 ""Controles""Tous\0    ""ON\0      ")

#define LEN_TRNMODE                    "\003"
#define TR_TRNMODE                     "OFF""+=\0"":="

#define LEN_TRNCHN                     "\003"
#define TR_TRNCHN                      "CH1CH2CH3CH4"

#define LEN_AUX_SERIAL_MODES           "\016"
#if defined(CLI) || defined(DEBUG)
#define TR_AUX_SERIAL_MODES            "Debug\0        ""Recopie Telem\0""T\200l\200m\200trie In\0""Ecolage SBUS\0 ""LUA\0          "
#else
#define TR_AUX_SERIAL_MODES            "OFF\0          ""Recopie Telem\0""T\200l\200m\200trie In\0""Ecolage SBUS\0 ""LUA\0          "
#endif

#define LEN_SWTYPES                    "\006"
#define TR_SWTYPES                     "Rien\0 ""Levier""2-POS\0""3-POS\0"

#define LEN_POTTYPES                   TR("\013","\017")
#define TR_POTTYPES                    TR("Rien\0      ""Pot av. ctr""Multipos\0  ""Pot\0       ", "Rien\0          ""Pot avec centre""Inter multi-pos""Potentiom\201tre\0")

#define LEN_SLIDERTYPES                "\006"
#define TR_SLIDERTYPES                 "Rien\0 ""Slider"

#define LEN_VLCD                       "\006"
#define TR_VLCD                        "NormalOptrex"

#define LEN_VPERSISTENT                "\014"
#define TR_VPERSISTENT                 "OFF\0        ""Vol\0        ""Reset Manuel"

#define LEN_COUNTRYCODES               TR("\002", "\006")
#define TR_COUNTRYCODES                TR("US""JP""EU", "USA\0  ""Japon\0""Europe")

#if defined(RADIO_FAMILY_TBS)
#define LEN_USBMODES                   TR("\006", "\010")
#define TR_USBMODES                    TR("Ask\0  ""Joyst\0""Agent\0""SDCard""Serial", "Ask\0    ""Joystick""Agent\0 ""Storage\0""Serial\0 ")
#else
#define LEN_USBMODES                   TR("\006", "\010")
#if defined(DEBUG)
#define TR_USBMODES                    TR("Popup\0""Joyst\0""SDCard""Telem\0""S\200rie\0", "Demander""Joystick""Stockage""Telem\0   ""S\200rie\0  ")
#else
#define TR_USBMODES                    TR("Popup\0""Joyst\0""SDCard""Telem\0""S\200rie\0", "Demander""Joystick""Stockage""Telem\0   ""S-Telem\0 ")
#endif
#endif

#define LEN_JACKMODES                  "\007"
#define TR_JACKMODES                   "Demander""Audio\0 ""Ecolage"

#define LEN_TELEMETRY_PROTOCOLS        "\017"
#define TR_TELEMETRY_PROTOCOLS         "FrSky S.PORT\0  ""FrSky D\0       ""FrSky D (cable)""TBS Crossfire\0 ""Spektrum\0      ""AFHDS2A IBUS\0  ""Multi Telemetry"

#define TR_MULTI_CUSTOM                "Perso"

#define LEN_VTRIMINC                   TR("\006", "\013")
#define TR_VTRIMINC                    TR("Expo\0 ""ExFin\0""Fin\0  ""Moyen\0""Gros\0 ","Exponentiel""Extra Fin\0 ""Fin\0       ""Moyen\0     ""Grossier\0  ")

#define LEN_VDISPLAYTRIMS              "\006"
#define TR_VDISPLAYTRIMS               "Non\0  ""Change""Oui\0"

#define LEN_VBEEPCOUNTDOWN             "\007"
#define TR_VBEEPCOUNTDOWN              "Aucun\0 ""Bips\0  ""Voix\0  Haptic\0"

#define LEN_VVARIOCENTER               "\006"
#define TR_VVARIOCENTER                "Tone\0 ""Silent"

#define LEN_CURVE_TYPES                "\010"
#define TR_CURVE_TYPES                 "Standard""Libre\0"

#define LEN_RETA123                    "\001"

#if defined(PCBHORUS)
  #define TR_RETA123                   "DPGA13245LR"
#elif defined(PCBX9E)
  #define TR_RETA123                   "DPGA1234LRLR"
#elif defined(PCBTARANIS) || defined(REVX)
  #define TR_RETA123                   "DPGA123LR"
#elif defined(PCBSKY9X)
  #define TR_RETA123                   "DPGA123a"
#else
  #define TR_RETA123                   "DPGA123"
#endif

#if defined(PCBSKY9X) && defined(REVX)
  #define LEN_VOUTPUT_TYPE             "\011"
  #define TR_VOUTPUT_TYPE              "OpenDrain""PushPull\0"
#endif

#define LEN_VCURVEFUNC                 "\003"
#define TR_VCURVEFUNC                  "---""x>0""x<0""|x|""f>0""f<0""|f|"

#define LEN_VMLTPX                     TR("\010", "\013")
#define TR_VMLTPX                      TR("Ajoute\0 ""Multipl.""Remplace", "Additionner""Multiplier\0""Remplacer\0")

#define LEN_VMLTPX2                    "\002"
#define TR_VMLTPX2                     "+=""*="":="

#define LEN_VMIXTRIMS                  "\003"

#if defined(PCBHORUS)
  #define TR_VMIXTRIMS                 "OFF""ON\0""Dir""Prf""Gaz""Ail""T5\0""T6\0"
#else
  #define TR_VMIXTRIMS                 "OFF""ON\0""Dir""Prf""Gaz""Ail"
#endif

#if LCD_W >= 212
  #define TR_CSWTIMER                  "Tempo"
  #define TR_CSWSTICKY                 "Bistb"
  #define TR_CSWRANGE                  "Plage"
  #define TR_CSWSTAY                   "Flanc"
#else
  #define TR_CSWTIMER                  "Temp\0"
  #define TR_CSWSTICKY                 "Bist\0"
    #define TR_CSWRANGE                "Zone\0"
    #define TR_CSWSTAY                 "Flnc\0"
#endif

  #define TR_CSWEQUAL                  "a=x\0 "

#define LEN_VCSWFUNC                   "\005"
#define TR_VCSWFUNC                    "---\0 " TR_CSWEQUAL "a\173x\0 ""a>x\0 ""a<x\0 " TR_CSWRANGE "|a|>x""|a|<x""ET\0  ""OU\0  ""OUX\0 " TR_CSWSTAY "a=b\0 ""a>b\0 ""a<b\0 ""\306}x\0 ""|\306|}x" TR_CSWTIMER TR_CSWSTICKY

#define LEN_TEXT_SIZE                  "\010"
#define TR_TEXT_SIZE                   "Standard""Tiny\0   ""Small\0  ""Mid\0    ""Double\0 "

#define LEN_VFSWFUNC                   "\015"

#if defined(VARIO)
  #define TR_VVARIO                    "Vario\0       "
#else
  #define TR_VVARIO                    "[Vario]\0     "
#endif

#if defined(AUDIO)
  #define TR_SOUND                     "Jouer son\0   "
#else
  #define TR_SOUND                     "Bip\0         "
#endif

#if defined(HAPTIC)
  #define TR_HAPTIC                    "Vibreur\0     "
#else
  #define TR_HAPTIC                    "[Vibreur]\0   "
#endif

#if defined(VOICE)
  #if defined(PCBSKY9X)
    #define TR_PLAY_TRACK              "Jouer\0       "
  #else
    #define TR_PLAY_TRACK              "Jouer fich\0  "
  #endif
  #define TR_PLAY_BOTH                 "Jouer les 2\0 "
  #define TR_PLAY_VALUE                "Lire valeur\0 "
#else
  #define TR_PLAY_TRACK                "[Jouer fich.]"
  #define TR_PLAY_BOTH                 "[Jouer les 2]"
  #define TR_PLAY_VALUE                "[Lire valeur]"
#endif

#define TR_SF_BG_MUSIC                 "Musique\0     ""Pause Musique"

#if defined(SDCARD)
  #define TR_SDCLOGS                   "Logs SD\0     "
#else
  #define TR_SDCLOGS                   "[Logs SD]\0   "
#endif

#if defined(GVARS)
  #define TR_ADJUST_GVAR               "Ajuster\0     "
#else
  #define TR_ADJUST_GVAR               "[AjusteGV]\0  "
#endif

#if defined(LUA)
  #define TR_SF_PLAY_SCRIPT            "Script Lua\0  "
#else
  #define TR_SF_PLAY_SCRIPT            "[Lua]\0       "
#endif

#if defined(DEBUG)
  #define TR_SF_TEST                   "Test\0        "
#else
  #define TR_SF_TEST
#endif

#if defined(OVERRIDE_CHANNEL_FUNCTION) && LCD_W >= 212
  #define TR_SF_SAFETY                 "Remplace\0    "
#elif defined(OVERRIDE_CHANNEL_FUNCTION)
  #define TR_SF_SAFETY                 "Rempl.\0      "
#else
  #define TR_SF_SAFETY                 "---\0         "
#endif

#define TR_SF_SCREENSHOT               "Photo Ecran\0 "
#define TR_SF_RACING_MODE              "Racing Mode\0 "
#define TR_SF_RESERVE                  "[reserve]\0   "

  #define TR_VFSWFUNC                  TR_SF_SAFETY "Ecolage\0     ""Trim instant.""Remise \202 0\0  ""D\200f.\0        " TR_ADJUST_GVAR "Volume\0      " "D\200fFailsafe\0 " "Test Port.\0  " "Bind\0        " TR_SOUND TR_PLAY_TRACK TR_PLAY_VALUE TR_SF_RESERVE TR_SF_PLAY_SCRIPT TR_SF_RESERVE TR_SF_BG_MUSIC TR_VVARIO TR_HAPTIC TR_SDCLOGS "R\200tro\200cl.\0   " TR_SF_SCREENSHOT TR_SF_RACING_MODE TR_SF_TEST

#define LEN_VFSWRESET                  TR("\004", "\012")

#define TR_FSW_RESET_TELEM             TR("T\200lm", "T\200l\200m\200trie")

#if LCD_W >= 212
  #define TR_FSW_RESET_TIMERS          "Chrono 1\0 ""Chrono 2\0 ""Chrono 3\0 "
#else
  #define TR_FSW_RESET_TIMERS          "Chr1""Chr2""Chr3"
#endif

#define TR_VFSWRESET                   TR(TR_FSW_RESET_TIMERS "Tout" TR_FSW_RESET_TELEM, TR_FSW_RESET_TIMERS "Tout\0     " TR_FSW_RESET_TELEM)

#define LEN_FUNCSOUNDS                 TR("\004", "\006")
#define TR_FUNCSOUNDS                  TR("Bp1\0""Bp2\0""Bp3\0""Wrn1""Wrn2""Chee""Rata""Tick""Sirn""Ring""SciF""Robt""Chrp""Tada""Crck""Alrm", "Beep1 ""Beep2 ""Beep3 ""Warn1 ""Warn2 ""Cheep ""Ratata""Tick  ""Siren ""Ring  ""SciFi ""Robot ""Chirp ""Tada  ""Crickt""AlmClk")

#define LEN_VTELEMCHNS                 "\004"

  #define TR_TELEM_RESERVE             TR("[--]", "[---]")
  #define TR_TELEM_TIME                TR("Heur", "H:M\0 ")
  #define TR_RAS                       TR("SWR\0", "SWR\0 ")
  #define TR_RX_BATT                   TR("BtRx", "BatRx")
  #define TR_A3_A4                     TR("A3\0 ""A4\0 ", "A3\0  ""A4\0  ")
  #define TR_A3_A4_MIN                 TR("A3-\0""A4-\0", "A3-\0 ""A4-\0 ")

#define TR_ASPD_MAX                    TR("ViA+", "VitA+")

#if LCD_W >= 212
  #define TR_TELEM_RSSI_RX             "RSSI\0"
#else
  #define TR_TELEM_RSSI_RX             TR("Rx\0 ", "Rx\0  ")
#endif

  #define TR_TELEM_TIMERS              TR("Chr1""Chr2""Chr3", "Chr1\0""Chr2\0""Chr3\0")

#define LENGTH_UNIT_IMP                "ft\0"
#define SPEED_UNIT_IMP                 "mph"
#define LENGTH_UNIT_METR               "m\0 "
#define SPEED_UNIT_METR                "kmh"

#define LEN_VUNITSSYSTEM             TR("\006", "\012")
#define TR_VUNITSSYSTEM              TR("M\200tr.\0""Imp\200r.", "M\200triques\0""Imp\200riales")
#define LEN_VTELEMUNIT               "\003"
#define TR_VTELEMUNIT                "-\0 ""V\0 ""A\0 ""mA\0""kts""m/s""f/s""kmh""mph""m\0 ""ft\0""@C\0""@F\0""%\0 ""mAh""W\0 ""mW\0""dB\0""rpm""g\0 ""@\0 ""rad""ml\0""fOz""mlm""Hz\0""mS\0""uS\0""km\0""dbm"

#define STR_V                          (STR_VTELEMUNIT+1)
#define STR_A                          (STR_VTELEMUNIT+4)

#define LEN_VTELEMSCREENTYPE         "\007"
#define TR_VTELEMSCREENTYPE          "Rien\0  ""Valeurs""Barres\0""Script\0"

#define LEN_GPSFORMAT                  "\004"
#define TR_GPSFORMAT                   "DMS\0""NMEA"

#define LEN2_VTEMPLATES                14
#define LEN_VTEMPLATES                 "\016"
#define TR_TEMPLATE_CLEAR_MIXES        "Suppr mixages\0"
#define TR_TEMPLATE_SIMPLE_4CH         "4 voies simple"
#define TR_TEMPLATE_STICKY_TCUT        "Coupure gaz\0  "
#define TR_TEMPLATE_VTAIL              "Empennage V\0  "
#define TR_TEMPLATE_DELTA              "Elevon\\Delta\0 "
#define TR_TEMPLATE_ECCPM              "eCCPM\0         "
#define TR_TEMPLATE_HELI               "Conf. H\200lico\0  "
#define TR_TEMPLATE_SERVO_TEST         "Test Servo\0    "

#define LEN_VSWASHTYPE                 "\004"
#define TR_VSWASHTYPE                  "--- ""120 ""120X""140 ""90\0"

#if defined(PCBHORUS)
  #define LEN_VKEYS                    "\005"
  #define TR_VKEYS                     "PGUP\0""PGDN\0""ENTER""MDL\0 ""RTN\0 ""TELE\0""SYS\0 "
#elif defined(PCBXLITE)
  #define LEN_VKEYS                    "\005"
  #define TR_VKEYS                     "Shift""Exit\0""Enter""Bas\0 ""Haut\0""Droit""Gauch"
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
  #define TR_VKEYS                     "Menu\0""Exit\0""Enter""Page\0""Plus\0""Moins"
#else
  #define LEN_VKEYS                    "\005"
  #define TR_VKEYS                     "Menu\0""Exit\0""Bas\0 ""Haut\0""Droit""Gauch"
#endif

#define LEN_VSWITCHES                  "\003"
#define LEN_VSRCRAW                    "\004"

#define TR_STICKS_VSRCRAW              "\307Dir""\307Prf""\307Gaz""\307Ail"

#if defined(PCBHORUS)
  #define TR_TRIMS_VSRCRAW             "\313Dir""\313Prf""\313Gaz""\313Ail""\313T5\0""\313T6\0"
#else
  #define TR_TRIMS_VSRCRAW             "\313Dir""\313Prf""\313Gaz""\313Ail"
#endif

#if defined(PCBHORUS)
  #define TR_TRIMS_SWITCHES            "\313Dg""\313Dd""\313Pb""\313Ph""\313Gb""\313Gh""\313Ag""\313Ad""\3135d""\3135u""\3136d""\3136u"
#else
  #define TR_TRIMS_SWITCHES            TR("tDg""tDd""tPb""tPh""tGb""tGh""tAg""tAd", "\313Dg""\313Dd""\313Pb""\313Ph""\313Gb""\313Gh""\313Ag""\313Ad")
#endif

#if defined(PCBSKY9X)
  #define TR_ROTARY_ENCODERS           "REa\0"
  #define TR_ROTENC_SWITCHES           "REa"
#else
  #define TR_ROTARY_ENCODERS
  #define TR_ROTENC_SWITCHES
#endif

#define TR_ON_ONE_SWITCHES             "ON\0""Un"

#if defined(GYRO)
  #define TR_GYR_VSRCRAW               "GyrX""GyrY"
#else
  #define TR_GYR_VSRCRAW
#endif

#if defined(HELI)
  #define TR_CYC_VSRCRAW               "CYC1""CYC2""CYC3"
#else
  #define TR_CYC_VSRCRAW               "[C1]""[C2]""[C3]"
#endif

#define TR_RESERVE_VSRCRAW           "[--]"
#define TR_EXTRA_VSRCRAW             "Batt""H:M\0""GPS\0" TR_RESERVE_VSRCRAW TR_RESERVE_VSRCRAW TR_RESERVE_VSRCRAW TR_RESERVE_VSRCRAW "Chr1""Chr2""Chr3"

#define LEN_VTMRMODES                  "\003"
#define TR_VTMRMODES                   "OFF""ON\0""GZs""GZ%""GZt"

#define LEN_VTRAINERMODES              "\022"
#define TR_VTRAINER_MASTER_JACK        "Ma\203tre/Jack\0      "
#define TR_VTRAINER_SLAVE_JACK         "El\201ve/Jack\0       "
#define TR_VTRAINER_MASTER_SBUS_MODULE "Ma\203tre/SBUS Module"
#define TR_VTRAINER_MASTER_CPPM_MODULE "Ma\203tre/CPPM Module"
#define TR_VTRAINER_MASTER_BATTERY     "Ma\203tre/S\200rie\0     "
#define TR_VTRAINER_BLUETOOTH          TR("Ma\203tre/BT\0        ""El\201ve/BT\0         ", "Ma\203tre/Bluetooth\0 ""El\201ve/Bluetooth\0  ")
#define TR_VTRAINER_MULTI              "Master/Multi\0     "
#define TR_VTRAINER_SPORT_SBUS         "Master/SBUS-Sport\0"

#define LEN_VFAILSAFE                  "\011"
#define TR_VFAILSAFE                   TR("Pas d\200f.\0""Maintien\0""Pr\200d\200f.\0 ""Pas d'imp""R\200cepteur", "Pas d\200f.\0""Maintien\0""Pr\200d\200fini""Pas d'imp""R\200cepteur")


#define LEN_VSENSORTYPES               "\012"
#define TR_VSENSORTYPES                "Perso\0    ""Calcul\200\0  "

#define LEN_VFORMULAS                  "\010"
#define TR_VFORMULAS                   "Addition""Moyenne\0""Min\0    ""Max\0    ""Multipl.""Totalise""El\200ment\0""Consomm.""Distance"

#define LEN_VPREC                      "\004"
#define TR_VPREC                       "0.--""0.0 ""0.00"

#define LEN_VCELLINDEX                 "\007"
#define TR_VCELLINDEX                  "Mini.\0 ""1\0     ""2\0     ""3\0     ""4\0     ""5\0     ""6\0     ""Maxi.\0 ""Diff.\0 "

#define LEN_GYROS                      "\004"
#define TR_GYROS                       "GyrX""GyrY"

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

#define TR_MENUWHENDONE                CENTER "\006" TR_ENTER " QUAND PRET"
#define TR_FREE                        "disp"
#define TR_DELETEMODEL                 "SUPPRIMER" BREAKSPACE "MODELE"
#define TR_COPYINGMODEL                "Copie..."
#define TR_MOVINGMODEL                 "D\200placement..."
#define TR_LOADINGMODEL                "Chargement..."
#define TR_NAME                        "Nom"
#define TR_MODELNAME                   TR("Nom mod\201le", "Nom du mod\201le")
#define TR_PHASENAME                   "Nom phase"
#define TR_MIXNAME                     TR("Nom ligne", "Nom du mixeur")
#define TR_INPUTNAME                   TR("Entr\200e", "Nom entr\200e")
#define TR_EXPONAME                    TR("Nom", "Nom ligne")
#define TR_BITMAP                      "Image du mod\201le"
#define TR_TIMER                       "Chrono "
#define TR_ELIMITS                     TR("Limites \200t.", "Limites \200tendues")
#define TR_ETRIMS                      TR("Trims \200t.", "Trims \200tendus")
#define TR_TRIMINC                     TR("Pas Trim", "Pas des trims")
#define TR_DISPLAY_TRIMS               TR("Aff. trims", "Affichage trims")
#define TR_TTRACE                      TR("Source gaz", INDENT "Source")
#define TR_TTRIM                       TR("Trim gaz", INDENT "Trim ralenti uniq.")
#define TR_TTRIM_SW                    TR("T-Trim-Sw", INDENT "Trim switch")
#define TR_BEEPCTR                     TR("Bips centr", "Bips centrage")
#define TR_USE_GLOBAL_FUNCS            TR("Fonc. glob.", "Fonctions globales")
#if defined(PCBSKY9X) && defined(REVX)
  #define TR_OUTPUT_TYPE               INDENT "Sortie"
#endif
#define TR_PROTOCOL                    TR("Proto.", "Protocole")
#define TR_PPMFRAME                    INDENT "Trame PPM"
#define TR_REFRESHRATE                 INDENT "P\200riode"
#define STR_WARN_BATTVOLTAGE           TR(INDENT "Signal est VBAT: ", INDENT "ATTENTION: signal est \202 VBAT: ")
#define TR_WARN_5VOLTS                 "Warning: output level is 5 volts"
#define TR_MS                          "ms"
#define TR_FREQUENCY                   INDENT "Fr\200quence"
#define TR_SWITCH                      TR("Inter", "Interrupteur")
#define TR_TRIMS                       "Trims"
#define TR_FADEIN                      "Fondu ON"
#define TR_FADEOUT                     "Fondu OFF"
#define TR_DEFAULT                     "(d\200faut)"
#define TR_CHECKTRIMS                  "\006V\200rif\012Trims"
#define OFS_CHECKTRIMS                 (9*FW)
#define TR_SWASHTYPE                   TR("Type de Plat.", "Type de plateau")
#define TR_COLLECTIVE                  TR("Collectif", "Source collectif")
#define TR_AILERON                     "Source cyc. lat."
#define TR_ELEVATOR                    "Source cyc. lon."
#define TR_SWASHRING                   TR("Limite Cycl.", "Limite du cyclique")
#define TR_ELEDIRECTION                TR("Inv. longitud.", "Inversion longitudinal")
#define TR_AILDIRECTION                TR("Inv. lat\200ral", "Inversion lat\200ral")
#define TR_COLDIRECTION                TR("Inv. collectif", "Inversion collectif")
#define TR_MODE                        "Mode"
#define TR_SUBTYPE                     INDENT "Sous-type"
#define TR_NOFREEEXPO                  "Max expos atteint!"
#define TR_NOFREEMIXER                 "Max mixages atteint!"
#define TR_SOURCE                      "Source"
#define TR_WEIGHT                      "Ratio"
#define TR_EXPO                        TR("Expo", "Exponentiel")
#define TR_SIDE                        "Cot\200"
#define TR_DIFFERENTIAL                "Diff\200rentiel"
#define TR_OFFSET                      "D\200calage"
#define TR_TRIM                        "Trim"
#define TR_DREX                        "DRex"
#define DREX_CHBOX_OFFSET              30
#define TR_CURVE                       "Courbe"
#define TR_FLMODE                      TR("Phase", "Phases")
#define TR_MIXWARNING                  "Alerte"
#define TR_OFF                         "OFF"
#define TR_ANTENNA                     "Antenne"
#define TR_NO_INFORMATION              TR("Pas d'info", "Pas d'information")
#define TR_MULTPX                      "Op\200ration"
#define TR_DELAYDOWN                   "Retard bas"
#define TR_DELAYUP                     "Retard haut"
#define TR_SLOWDOWN                    "Ralenti bas"
#define TR_SLOWUP                      "Ralenti haut"
#define TR_MIXES                       "MIXEUR"
#define TR_CV                          "CB"
#define TR_GV                          TR("G", "VG")
#define TR_ACHANNEL                    "A"
#define TR_RANGE                       INDENT "Plage"
#define TR_CENTER                      INDENT "Centre"
#define TR_BAR                         "Barre"
#define TR_ALARM                       INDENT "Alarme"
#define TR_USRDATA                     "Donn\200es"
#define TR_BLADES                      INDENT "Pales/Poles"
#define TR_SCREEN                      "Ecran "
#define TR_SOUND_LABEL                 "Son"
#define TR_LENGTH                      INDENT "Dur\200e"
#define TR_BEEP_LENGTH                 INDENT "Dur\200e bips"
#define TR_SPKRPITCH                   INDENT "Tonalit\200"
#define TR_HAPTIC_LABEL                "Vibreur"
#define TR_HAPTICSTRENGTH              INDENT "Force"
#define TR_GYRO_LABEL                  "Gyro"
#define TR_GYRO_OFFSET                 "D\200calage"
#define TR_GYRO_MAX                    "Max"
#define TR_CONTRAST                    "Contraste"
#define TR_ALARMS_LABEL                "Alarmes"
#define TR_BATTERY_RANGE               "Plage batterie"
#define TR_BATTERYWARNING              TR(INDENT "Batterie", INDENT "Batterie faible")
#define TR_INACTIVITYALARM             INDENT "Inactivit\200"
#define TR_MEMORYWARNING               INDENT "M\200moire pleine"
#define TR_ALARMWARNING                TR(INDENT "Silence", INDENT "Sons d\200sactiv\200s")
#define TR_RSSISHUTDOWNALARM           TR(INDENT "RSSI extinct.", INDENT "V\200rif RSSI \202 l'extinction")
#define TR_MODEL_STILL_POWERED         TR("Mod\201le allum\200", "Mod\201le encore allum\200")
#define TR_MODEL_SHUTDOWN              "Eteindre ?"
#define TR_PRESS_ENTER_TO_CONFIRM      "Presser [Enter] pour confirmer"
#define TR_THROTTLE_LABEL              "Gaz"
#define TR_THROTTLEREVERSE             TR("Inv. gaz", INDENT "Inversion gaz")
#define TR_MINUTEBEEP                  TR("Bip min.", "Annonces minutes")
#define TR_BEEPCOUNTDOWN               TR(INDENT "Bip fin", INDENT "Compte \202 rebours")
#define TR_PERSISTENT                  TR(INDENT "Persist.", INDENT "Persistant")
#define TR_BACKLIGHT_LABEL             "R\200tro\200clairage"
#define TR_GHOST_MENU_LABEL            "GHOST MENU"
#define TR_STATUS                      "Status"
#define TR_RAW_12BITS                  "Raw 12 bits"
#define TR_BLDELAY                     INDENT "Dur\200e"
#define TR_BLONBRIGHTNESS              INDENT "Luminosit\200 ON"
#define TR_BLOFFBRIGHTNESS             INDENT "Luminosit\200 OFF"
#define TR_KEYS_BACKLIGHT              "Keys backlight"
#define TR_BLCOLOR                     INDENT "Couleur"
#define TR_SPLASHSCREEN                "Logo d'accueil"
#define TR_PWR_ON_DELAY                "D\200lai btn ON"
#define TR_PWR_OFF_DELAY               "D\200lai btn OFF"
#define TR_THROTTLEWARNING             TR(INDENT "Alerte gaz", INDENT "Alerte gaz")
#define TR_SWITCHWARNING               TR(INDENT "Alerte int", INDENT "Pos. interrupteurs")
#define TR_POTWARNINGSTATE             TR(INDENT "Pot&Slid.", INDENT "Pots & sliders")
#define TR_SLIDERWARNING               TR(INDENT "Slid. pos.", INDENT "Slider positions")
#define TR_POTWARNING                  TR(INDENT "Alerte pot", INDENT "Pos. potentios")
#define TR_TIMEZONE                    "Fuseau horaire"
#define TR_ADJUST_RTC                  TR("Ajust. RTC", INDENT "Ajust. heure auto")
#define TR_GPS                         "GPS"
#define TR_RXCHANNELORD                TR("Ordre voies RX","Ordre des voies pr\200f\200r\200")
#define TR_STICKS                      "Manches"
#define TR_POTS                        "Potentiom\201tres"
#define TR_SWITCHES                    "Inters"
#define TR_SWITCHES_DELAY              "D\200lai inter son"
#define TR_SLAVE                       "El\201ve"
#define TR_MODESRC                     "Mode\006% Source"
#define TR_MULTIPLIER                  "Multiplieur"
#define TR_CAL                         "Cal"
#define TR_VTRIM                       "Trim- +"
#define TR_BG                          "BG:"
#if defined(PCBHORUS)
  #define TR_MENUTOSTART               "Presser [Enter] pour commencer"
  #define TR_SETMIDPOINT               "Centrer manches/pots/sliders puis [Enter]"
  #define TR_MOVESTICKSPOTS            "Bouger manches/pots/sliders puis [Enter]"
#elif defined(COLORLCD)
  #define TR_MENUTOSTART               TR_ENTER " POUR DEBUT"
  #define TR_SETMIDPOINT               "REGLER NEUTRES"
  #define TR_MOVESTICKSPOTS            "BOUGER STICKS/POTS"
#else
  #define TR_MENUTOSTART               CENTER"\006" TR_ENTER " POUR DEBUT"
  #define TR_SETMIDPOINT               CENTER"\010REGLER NEUTRES"
  #define TR_MOVESTICKSPOTS            CENTER"\004BOUGER STICKS/POTS"
#endif
#define TR_RXBATT                      "Batt.RX"
#define TR_TXnRX                       "Tx:\0Rx:"
#define OFS_RX                         4
#define TR_ACCEL                       "Acc:"
#define TR_NODATA                      CENTER "NO DATA"
#define TR_US                          "us"
#define TR_TMR1LATMINUS                "Tmr1Lat min\037\124us"

#define TR_TMIXMAXMS                   "Tmix max"
#define TR_FREE_STACK                  "Free stack"

#define TR_MENUTORESET                 TR_ENTER" pour reset"
#define TR_PPM_TRAINER                 "TR"
#define TR_CH                          "CH"
#define TR_MODEL                       "MODELE"
#define TR_FM                          "PV"
#define TR_MIX                         "MIX"
#define TR_EEPROMLOWMEM                "EEPROM pleine!"
#define TR_ALERT                       "\014ALERTE"
#define TR_PRESSANYKEYTOSKIP           "Touche pour ignorer"
#define TR_THROTTLENOTIDLE             "Gaz pas \202 z\200ro"
#define TR_ALARMSDISABLED              "Alarmes D\200sactiv\200es"
#define TR_PRESSANYKEY                 TR("Touche pour continuer", "Touche pour continuer")
#define TR_BADEEPROMDATA               "EEPROM corrompue"
#define TR_BAD_RADIO_DATA              "R\200glages radio corrompus"
#define TR_EEPROMFORMATTING            "Formatage EEPROM"
#define TR_STORAGE_FORMAT              "Pr\200paration stockage"
#define TR_EEPROMOVERFLOW              "D\200passement EEPROM"
#define TR_MENURADIOSETUP              "CONFIG RADIO"
#define TR_MENUDATEANDTIME             "DATE ET HEURE"
#define TR_MENUTRAINER                 "ECOLAGE"
#define TR_MENUSPECIALFUNCS            "FONCTIONS GLOBALES"
#define TR_MENUVERSION                 "VERSION"
#define TR_MENU_RADIO_SWITCHES         TR("INTERS", "TEST INTERRUPTEURS")
#define TR_MENU_RADIO_ANALOGS          TR("ANAS", "ENTREES ANALOGIQUES")
#define TR_MENUCALIBRATION             "CALIBRATION"
#if defined(COLORLCD)
  #define TR_TRIMS2OFFSETS             "Trims => Subtrims"
#else
  #define TR_TRIMS2OFFSETS             "\006Trims => Subtrims"
#endif
#define TR_CHANNELS2FAILSAFE          "Channels=>Failsafe"
#define TR_CHANNEL2FAILSAFE            "Channel=>Failsafe"
#define TR_MENUMODELSEL                "MODELES"
#define TR_MENUSETUP                   TR("CONF. MODELE", "CONFIGURATION")
#define TR_MENUFLIGHTMODE              "PHASE DE VOL"
#define TR_MENUFLIGHTMODES             "PHASES DE VOL"
#define TR_MENUHELISETUP               TR("CONF.HELI", "CONFIGURATION HELICO")

  #define TR_MENUINPUTS                "ENTREES"
  #define TR_MENULIMITS                "SORTIES"

#define TR_MENUCURVES                  "COURBES"
#define TR_MENUCURVE                   "COURBE"
#define TR_MENULOGICALSWITCH           "INTER LOG."
#define TR_MENULOGICALSWITCHES         TR("INTERS LOG.", "INTERS LOGIQUES")
#define TR_MENUCUSTOMFUNC              TR("FONCTIONS SPEC.", "FONCTIONS SPECIALES")
#define TR_MENUCUSTOMSCRIPTS           "SCRIPTS PERSOS"
#define TR_MENUTELEMETRY               "TELEMESURE"
#define TR_MENUTEMPLATES               "GABARITS"
#define TR_MENUSTAT                    TR("STATS", "STATISTIQUES")
#define TR_MENUDEBUG                   "DEBUG"
#define TR_MONITOR_CHANNELS1           "VOIES 1-8"
#define TR_MONITOR_CHANNELS2           "VOIES 9-16"
#define TR_MONITOR_CHANNELS3           "VOIES 17-24"
#define TR_MONITOR_CHANNELS4           "VOIES 25-32"
#define TR_MONITOR_SWITCHES            "INTERS LOGIQUES"
#define TR_MONITOR_OUTPUT_DESC         "Sorties"
#define TR_MONITOR_MIXER_DESC          "Mixeurs"
#define TR_RECEIVER_NUM                TR("NumRx", "No. r\200cepteur")
#define TR_RECEIVER                    "R\200cept."
#define TR_MULTI_RFTUNE                TR(INDENT "Ajust.fr\200q", INDENT "Ajustement fr\200q.")
#define TR_MULTI_RFPOWER               "RF power"
#define TR_MULTI_WBUS                  "Output"
#define TR_MULTI_TELEMETRY             "T\200l\200m\200trie"
#define TR_MULTI_VIDFREQ               TR(INDENT "Fr\200q. vid\200o", INDENT "Fr\200quence vid\200o")
#define TR_RFPOWER                     TR("Puiss. RF", "Puissance RF")
#define TR_MULTI_FIXEDID               "ID fixe"
#define TR_MULTI_OPTION                TR(INDENT "Option", INDENT "Option perso")
#define TR_MULTI_AUTOBIND              TR(INDENT "Bind voie", INDENT "Bind sur voie")
#define TR_DISABLE_CH_MAP              TR("No Ch. map", "D\200sact r\200org voies")
#define TR_DISABLE_TELEM               TR("No Telem", "D\200sact T\200l\200m\200trie")
#define TR_MULTI_DSM_AUTODTECT         TR(INDENT "Autod\200t.", INDENT "Autod\200tection")
#define TR_MULTI_LOWPOWER              TR(INDENT "Basse puis.", INDENT "Mode basse puiss.")
#define TR_MULTI_LNA_DISABLE            INDENT "LNA disable"
#define TR_MODULE_TELEMETRY            TR(INDENT "S.Port", INDENT "Lien S.Port")
#define TR_MODULE_TELEM_ON             TR("ON", "Actif")
#define TR_DISABLE_INTERNAL            TR("D\200sact intRF", "D\200sact. RF interne")
#define TR_MODULE_NO_SERIAL_MODE       TR("Mode s\200rie?", "Pas en mode s\200rie")
#define TR_MODULE_NO_INPUT             TR("Pas de sign.", "Aucun signal s\200rie")
#define TR_MODULE_NO_TELEMETRY         TR3("Pas de t\200lm.", "T\200l\200m\200trie absente", "T\200l\200m\200trie absente(act. MULTI_TELEMETRY)")
#define TR_MODULE_WAITFORBIND          "Binder d'abord"
#define TR_MODULE_BINDING              "Bind..."
#define TR_MODULE_UPGRADE_ALERT              "Mise \202 jour requise"
#define TR_MODULE_UPGRADE              TR("Upg. advised", "Module update recommended")
#define TR_PULSE_RATE                  "Pulse rate"
#define TR_LINK_SPEED                  "Link speed"
#define TR_REBIND                      "Rebind requis"
#define TR_REG_OK                      "Enregistr. ok"
#define TR_BIND_OK                     "Bind ok"
#define TR_BINDING_CH1_8_TELEM_ON      "Ch1-8 T\200lem ON"
#define TR_BINDING_CH1_8_TELEM_OFF     "Ch1-8 T\200lem OFF"
#define TR_BINDING_CH9_16_TELEM_ON     "Ch9-16 T\200lem ON"
#define TR_BINDING_CH9_16_TELEM_OFF    "Ch9-16 T\200lem OFF"
#define TR_PROTOCOL_INVALID            TR("S\200l. invalide", "Protocole invalide")
#define TR_MODULE_STATUS               TR(INDENT "Etat", INDENT "Etat module")
#define TR_MODULE_SYNC                 TR(INDENT "Sync", INDENT "Proto Sync Status")
#define TR_MULTI_SERVOFREQ             TR(INDENT "Fr\200q.servo", INDENT "Fr\200quence servos")
#define TR_MULTI_MAX_THROW             TR("Max. Throw", "Enable max. throw")
#define TR_MULTI_RFCHAN                TR("RF Channel", "Select RF channel")
#define TR_SYNCMENU                    "Sync [MENU]"
#define TR_LIMIT                       INDENT "Limite"
#define TR_MINRSSI                     "RSSI Min."
#define TR_LATITUDE                    "Latitude"
#define TR_LONGITUDE                   "Longitude"
#define TR_GPSCOORD                    TR("Coordonn\200es", "Coordonn\200es GPS")
#define TR_VARIO                       TR("Vario", "Variom\201tre")
#define TR_PITCH_AT_ZERO               INDENT "Tonalit\200 z\200ro"
#define TR_PITCH_AT_MAX                INDENT "Tonalit\200 max"
#define TR_REPEAT_AT_ZERO              TR(INDENT "Interv. z\200ro", INDENT "Intervalle au z\200ro")
#define TR_SHUTDOWN                    "ARRET EN COURS"
#define TR_SAVEMODEL                   "Sauvegarde mod\201le..."
#define TR_BATT_CALIB                  "Calib. batt"
#define TR_CURRENT_CALIB               "Calib. cour"
#define TR_VOLTAGE                     TR(INDENT "Tension",INDENT "Source tension")
#define TR_CURRENT                     TR(INDENT "Courant",INDENT "Source courant")
#define TR_SELECT_MODEL                "S\200lect. mod\201le"
#define TR_SELECT_MODE                 "Select mode"
#define TR_CREATE_CATEGORY             "Cr\200er une cat\200gorie"
#define TR_RENAME_CATEGORY             "Renommer la cat\200gorie"
#define TR_DELETE_CATEGORY             "Supprimer la cat\200gorie"
#define TR_CREATE_MODEL                "Cr\200er mod\201le"
#define TR_DUPLICATE_MODEL             "Dupliquer mod\201le"
#define TR_COPY_MODEL                  "Copier mod\201le"
#define TR_MOVE_MODEL                  "D\200placer mod\201le"
#define TR_BACKUP_MODEL                "Archiver mod\201le"
#define TR_DELETE_MODEL                "Supprimer mod\201le"
#define TR_RESTORE_MODEL               "Restaurer mod\201le"
#define TR_DELETE_ERROR                "Effacement impossible"
#define TR_CAT_NOT_EMPTY               "Categorie non vide"
#define TR_SDCARD_ERROR                "Erreur carte SD"
#define TR_NO_SDCARD                   "Carte SD indisponible"
#define TR_WAITING_FOR_RX              "Attente du RX..."
#define TR_WAITING_FOR_TX              "Attente du TX..."
#define TR_WAITING_FOR_MODULE          TR("Waiting module", "Waiting for module...")
#define TR_NO_TOOLS                    "Pas d'outils dispo"
#define TR_NORMAL                      "Normal"
#define TR_NOT_INVERTED                "Non inv"
#define TR_NOT_CONNECTED               "!Connect\200"
#define TR_CONNECTED                   "Connect\200"
#define TR_FLEX_915                    "Flex 915MHz"
#define TR_FLEX_868                    "Flex 868MHz"
#define TR_16CH_WITHOUT_TELEMETRY      TR("16CH sans t\200l\200m.", "16CH sans t\200l\200m\200trie")
#define TR_16CH_WITH_TELEMETRY         TR("16CH avec t\200l\200m.", "16CH avec t\200l\200m\200trie")
#define TR_8CH_WITH_TELEMETRY          TR("8CH avec t\200l\200m.", "8CH avec t\200l\200m\200trie")
#define TR_EXT_ANTENNA                 "Antenne Ext."
#define TR_PIN                         "Pin"
#define TR_UPDATE_RX_OPTIONS           "Sauver options RX?"
#define TR_UPDATE_TX_OPTIONS           "Sauver options TX?"
#define TR_MODULES_RX_VERSION          "Versions modules / RX"
#define TR_MENU_MODULES_RX_VERSION     "VERSIONS MODULES / RX "
#define TR_MENU_FIRM_OPTIONS           "OPTIONS FIRMWARE"
#define TR_GYRO                        "Gyro"
#define TR_STICKS_POTS_SLIDERS         "Manches/Pots/leviers"
#define TR_PWM_STICKS_POTS_SLIDERS     "Manches PWM/Pots/leviers"
#define TR_RF_PROTOCOL                 "Protocole RF"
#define TR_MODULE_OPTIONS              "Options module"
#define TR_POWER                       "Puissance"
#define TR_NO_TX_OPTIONS               "Pas d'options TX"
#define TR_RTC_BATT                    "Pile RTC"
#define TR_POWER_METER_EXT             "Puissancem\201tre (EXT)"
#define TR_POWER_METER_INT             "Puissancem\201tre (INT)"
#define TR_SPECTRUM_ANALYSER_EXT       TR("Spectre (EXT)", "Analyseur spectre (EXT)")
#define TR_SPECTRUM_ANALYSER_INT       TR("Spectre (INT)", "Analyseur spectre (INT)")
#define TR_SDCARD_FULL                 "Carte SD pleine"
#define TR_NEEDS_FILE                  "NEEDS FILE"
#define TR_EXT_MULTI_SPEC              "opentx-inv"
#define TR_INT_MULTI_SPEC              "stm-opentx-noinv"
#define TR_INCOMPATIBLE                "Incompatible"
#define TR_WARNING                     "ALERTE"
#define TR_EEPROMWARN                  "EEPROM"
#define TR_STORAGE_WARNING             "STOCKAGE"
#define TR_EEPROM_CONVERTING           "Conversion EEPROM"
#define TR_THROTTLEWARN                "GAZ"
#define TR_ALARMSWARN                  "SON"
#define TR_SWITCHWARN                  TR("INTERS","CONTROLES")
#define TR_FAILSAFEWARN                "FAILSAFE"
#define TR_TEST_WARNING                 TR("TESTING", "TEST BUILD")
#define TR_TEST_NOTSAFE                 "Version de test uniq."
#define TR_WRONG_SDCARDVERSION         "Version requise: "
#define TR_WARN_RTC_BATTERY_LOW        "Batterie RTC faible"
#define TR_WARN_MULTI_LOWPOWER         "Mode basse puis."
#define TR_BATTERY                     "BATTERIE"
#define TR_WRONG_PCBREV                "PCB incorrect d\200tect\200"
#define TR_EMERGENCY_MODE              "MODE SECOURS"
#define TR_PCBREV_ERROR                "Erreur PCB"
#define TR_NO_FAILSAFE                 TR3("Failsafe pas d\200f.", "Failsafe pas d\200f.", "Failsafe pas d\200fini")
#define TR_KEYSTUCK                    "Touche bloqu\200e"
#define TR_INVERT_THR                  "Inverser gaz?"
#define TR_SPEAKER_VOLUME              INDENT "Volume"
#define TR_LCD                         "Afficheur"
#define TR_BRIGHTNESS                  INDENT "Luminosit\200"
#define TR_CPU_TEMP                    "Temp. CPU\016>"
#define TR_CPU_CURRENT                 "Courant\022>"
#define TR_CPU_MAH                     "Consomm."
#define TR_COPROC                      "CoProc."
#define TR_COPROC_TEMP                 "Temp. MB \016>"
#define TR_CAPAWARNING                 INDENT "Capacit\200 Basse"
#define TR_TEMPWARNING                 INDENT "Surchauffe"
#define TR_FUNC                        "Fonction"
#define TR_V1                          "V1"
#define TR_V2                          "V2"
#define TR_DURATION                    "Dur\200e"
#define TR_DELAY                       "D\200lai"
#define TR_SD_CARD                     "Carte SD"
#define TR_SDHC_CARD                   "Carte SD-HC"
#define TR_NO_SOUNDS_ON_SD             "Aucun son sur SD"
#define TR_NO_MODELS_ON_SD             "Aucun mod\201le SD"
#define TR_NO_BITMAPS_ON_SD            "Aucun Bitmap SD"
#define TR_NO_SCRIPTS_ON_SD            "Aucun Script SD"
#define TR_SCRIPT_SYNTAX_ERROR          TR("Erreur syntaxe", "Erreur syntaxe script")
#define TR_SCRIPT_PANIC                "Script bloqu\200"
#define TR_SCRIPT_KILLED               "Script interrompu"
#define TR_SCRIPT_ERROR                "Erreur inconnue"
#define TR_PLAY_FILE                   "Lire"
#define TR_DELETE_FILE                 "Supprimer"
#define TR_COPY_FILE                   "Copier"
#define TR_RENAME_FILE                 "Renommer"
#define TR_ASSIGN_BITMAP               "S\200lectionner image"
#define TR_ASSIGN_SPLASH               "Logo d'accueil"
#define TR_EXECUTE_FILE                "Ex\200cuter"
#define TR_REMOVED                     " supprim\200"
#define TR_SD_INFO                     "Information"
#define TR_SD_FORMAT                   "Formater"
#define TR_NA                          "N/D"
#define TR_HARDWARE                    "MATERIEL"
#define TR_FORMATTING                  "Formatage..."
#define TR_TEMP_CALIB                  "Calib. temp"
#define TR_TIME                        "Heure"
#define TR_MAXBAUDRATE                 "Max bauds"
#define TR_BLUETOOTH                   "Bluetooth"
#define TR_BLUETOOTH_DISC              "D\200couvrir"
#define TR_BLUETOOTH_INIT              "Init"
#define TR_BLUETOOTH_DIST_ADDR         "Addr dist."
#define TR_BLUETOOTH_LOCAL_ADDR        "Addr locale"
#define TR_BLUETOOTH_PIN_CODE          "Code PIN"
#define TR_BAUDRATE                    "Baudrate BT"
#define LEN_BLUETOOTH_MODES            "\012"
#if defined(PCBX9E)
#define TR_BLUETOOTH_MODES             "---\0      ""Activ\200\0   "
#else
#define TR_BLUETOOTH_MODES             "---\0      ""T\200l\200m\200trie""Ecolage\0"
#endif
#define TR_SD_INFO_TITLE               "INFO SD"
#define TR_SD_TYPE                     "Type:"
#define TR_SD_SPEED                    "Vitesse:"
#define TR_SD_SECTORS                  "Secteurs:"
#define TR_SD_SIZE                     "Taille:"
#define TR_TYPE                        INDENT "Type"
#define TR_GLOBAL_VARS                 "Variables Globales"
#define TR_GVARS                       "V. GLOBALES"
#define TR_GLOBAL_VAR                  "Variable globale"
#define TR_MENUGLOBALVARS              "VARIABLES GLOBALES"
#define TR_OWN                         "Pers"
#define TR_DATE                        "Date"
#define TR_MONTHS                      { "Jan", "F\200v", "Mar", "Avr", "Mai", "Jun", "Jul", "Aou", "Sep", "Oct", "Nov", "Dec" }
#define TR_ROTARY_ENCODER              "Enc.Rot."
#define TR_INVERT_ROTARY               "Invert Rotary"
#define TR_CHANNELS_MONITOR            "MONITEUR CANAUX"
#define TR_MIXERS_MONITOR              "MONITEUR MIXAGES "
#define TR_PATH_TOO_LONG               "Chemin trop long"
#define TR_VIEW_TEXT                   "Voir texte"
#define TR_FLASH_BOOTLOADER            "Flasher BootLoader"
#define TR_FLASH_EXTERNAL_DEVICE       TR("Flasher S.Port", "Flasher S.Port externe")
#define TR_FLASH_RECEIVER_OTA          "Flasher RX OTA"
#define TR_FLASH_RECEIVER_BY_EXTERNAL_MODULE_OTA "Flash RX by ext. OTA"
#define TR_FLASH_RECEIVER_BY_INTERNAL_MODULE_OTA "Flash RX by int. OTA"
#define TR_FLASH_FLIGHT_CONTROLLER_BY_EXTERNAL_MODULE_OTA "Flash FC by ext. OTA"
#define TR_FLASH_FLIGHT_CONTROLLER_BY_INTERNAL_MODULE_OTA "Flash FC by int. OTA"
#define TR_FLASH_BLUETOOTH_MODULE      TR("Flash module BT", "Flash module Bluetooth")
#define TR_FLASH_POWER_MANAGEMENT_UNIT "Flasher pwr mngt unit"
#define TR_CURRENT_VERSION             "Version courante :"
#define TR_HW_REV                      "HW Rev"
#define TR_FLASH_INTERNAL_MODULE       TR("Flasher module int.", "Flasher module interne")
#define TR_FLASH_INTERNAL_MULTI        TR("Flash Int. Multi", "Flash Internal Multi")
#define TR_FLASH_EXTERNAL_MODULE       TR("Flasher module ext.", "Flasher module externe")
#define TR_FLASH_EXTERNAL_MULTI        TR("Flash Ext. Multi", "Flash External Multi")
#define TR_FLASH_EXTERNAL_ELRS         TR("Flash Ext. ELRS", "Flash External ELRS")
#define TR_FIRMWARE_UPDATE_ERROR       TR("Erreur m\202j FW","Erreur de mise \202 jour")
#define TR_FIRMWARE_UPDATE_SUCCESS     "Flash ok"
#define TR_WRITING                     "Ecriture..."
#define TR_CONFIRM_FORMAT              "Confirmer Formatage?"
#define TR_INTERNALRF                  "HF interne"
#define TR_INTERNAL_MODULE             TR("Module int.","Module interne")
#define TR_EXTERNAL_MODULE             TR("Module ext.","Module externe")
#define TR_OPENTX_UPGRADE_REQUIRED     "M\202j OpenTX n\200cessaire"
#define TR_TELEMETRY_DISABLED          "T\200l\200m. d\200sactiv\200e"
#define TR_MORE_OPTIONS_AVAILABLE      "Autres options disponibles"
#define TR_NO_MODULE_INFORMATION       "Pas d'info module"
#define TR_EXTERNALRF                  "HF externe"
#define TR_FAILSAFE                    TR(INDENT "Failsafe", INDENT "Type failsafe")
#define TR_FAILSAFESET                 "REGLAGES FAILSAFE"
#define TR_REG_ID                      TR("ID Enr.", "ID Enregistr.")
#define TR_OWNER_ID                    "ID Radio"
#define TR_PINMAPSET                   "PINMAP"
#define TR_HOLD                        "Hold"
#define TR_HOLD_UPPERCASE              "HOLD"
#define TR_NONE                        "None"
#define TR_NONE_UPPERCASE              "NONE"
#define TR_MENUSENSOR                  "CAPTEUR"
#define TR_POWERMETER_PEAK             "Pic"
#define TR_POWERMETER_POWER            "Puiss"
#define TR_POWERMETER_ATTN             "Attn"
#define TR_POWERMETER_FREQ             "Freq."
#define TR_MENUTOOLS                   "OUTILS"
#define TR_TURN_OFF_RECEIVER           "Eteindre r\200cept."
#define TR_STOPPING                    "Arret..."
#define TR_MENU_SPECTRUM_ANALYSER      "ANALYSEUR SPECTRE"
#define TR_MENU_POWER_METER            "MESURE PUISS."
#define TR_SENSOR                      "CAPTEUR"
#define TR_COUNTRYCODE                 TR("Zone g\200o.", "Zone g\200ographique")
#define TR_USBMODE                     "Mode USB"
#define TR_JACKMODE                    "Mode Jack"
#define TR_VOICELANG                   TR("Langue voix", "Langue annonces vocales")
#define TR_UNITSSYSTEM                 "Unit\200s"
#define TR_EDIT                        "Editer"
#define TR_INSERT_BEFORE               "Ins\200rer avant"
#define TR_INSERT_AFTER                "Ins\200rer apr\201s"
#define TR_COPY                        "Copier"
#define TR_MOVE                        "D\200placer"
#define TR_PASTE                       "Coller"
#define TR_DELETE                      "Supprimer"
#define TR_INSERT                      "Ins\200rer"
#define TR_RESET_FLIGHT                TR("R\200init. vol", "R\200initialiser vol")
#define TR_RESET_TIMER1                TR("R\200init. Timer1", "R\200initialiser Timer1")
#define TR_RESET_TIMER2                TR("R\200init. Timer2", "R\200initialiser Timer2")
#define TR_RESET_TIMER3                TR("R\200init. Timer3", "R\200initialiser Timer3")
#define TR_RESET_TELEMETRY             TR("R\200init. T\200l\200m.", "R\200init. T\200l\200m\200trie")
#define TR_STATISTICS                  "Statistiques"
#define TR_ABOUT_US                    "A propos"
#define TR_USB_JOYSTICK                "USB Joystick (HID)"
#define TR_USB_MASS_STORAGE            "Stockage USB (SD)"
#define TR_USB_SERIAL                  "Port s\200rie (Debug)"
#define TR_USB_TELEMETRY               "USB Telem mirror"
#define TR_SETUP_SCREENS               "Configuration \200crans"
#define TR_MONITOR_SCREENS             "Moniteurs"
#define TR_AND_SWITCH                  "ET suppl."
#define TR_SF                          "FS"
#define TR_GF                          "FG"
#define TR_SPEAKER                     INDENT "Haut-p."
#define TR_BUZZER                      INDENT "Bipeur"
#define TR_BYTES                       "bytes"
#define TR_MODULE_BIND                 BUTTON(TR("Bnd", "Bind"))
#define TR_POWERMETER_ATTN_NEEDED      "Att\200nuateur requis"
#define TR_PXX2_SELECT_RX              "S\200lect RX..."
#define TR_PXX2_DEFAULT                "<d\200faut>"
#define TR_BT_SELECT_DEVICE            "S\200lect appareil"
#define TR_DISCOVER                    "D\200couvrir"
#define TR_BUTTON_INIT                 BUTTON("Init")
#define TR_WAITING                     "Attente..."
#define TR_RECEIVER_DELETE             "Suppr r\200cept.?"
#define TR_RECEIVER_RESET              "Reset r\200cept.?"
#define TR_SHARE                       "Partg"
#define TR_BIND                        "Bind"
#define TR_REGISTER                    TR("Enr", "Enregistr.")
#define TR_MODULE_RANGE                BUTTON(TR("Prt", "Port."))
#define TR_RECEIVER_OPTIONS            TR("OPTIONS REC.", "OPTIONS RECEPTEUR")
#define TR_DEL_BUTTON                  BUTTON(TR("Eff", "Effacer"))
#define TR_RESET_BTN                   BUTTON("RAZ")
#define TR_DEBUG                       "Debug"
#define TR_KEYS_BTN                    BUTTON(TR("Btns","Inters"))
#define TR_ANALOGS_BTN                 BUTTON(TR("Anas", "Analogs"))
#define TR_TOUCH_NOTFOUND              "Touch hardware not found"
#define TR_TOUCH_EXIT                  "Touch screen to exit"
#define TR_CALIBRATION                 "Calibration"
#define TR_SET                         BUTTON("D\200f")
#define TR_TRAINER                     "Ecolage"
#define TR_CHANS                       "Chans"
#define TR_ANTENNAPROBLEM              CENTER "Antenne radio d\200fect.!"
#define TR_MODELIDUSED                 TR("ID affect\200 \202 :", "No de r\200cepteur utilis\200 par :")
#define TR_MODULE                      "Module"
#define TR_RX_NAME                     "Nom Rx"
#define TR_TELEMETRY_TYPE              TR("Type t\200l.", "Type t\200l\200m\200trie")
#define TR_TELEMETRY_SENSORS           "Capteurs"
#define TR_VALUE                       "Valeur"
#define TR_TOPLCDTIMER                 "Timer LCD haut"
#define TR_UNIT                        "Unit\200"
#define TR_TELEMETRY_NEWSENSOR         TR(INDENT"Nouveau capteur...", INDENT "Ajout d'un nouveau capteur...")
#define TR_CHANNELRANGE                TR(INDENT "Canaux", INDENT "Plage de canaux")
#define TR_AFHDS3_RX_FREQ              TR("RX freq.", "RX frequency")
#define TR_AFHDS3_ONE_TO_ONE_TELEMETRY TR("Unicast/Tel.", "Unicast/Telemetry")
#define TR_AFHDS3_ONE_TO_MANY          "Multicast"
#define TR_AFHDS3_ACTUAL_POWER         TR("Act. pow", "Actual power")
#define TR_AFHDS3_POWER_SOURCE         TR("Power src.", "Power source")
#define TR_ANTENNACONFIRM1             "Vraiment changer?"
#if defined(PCBX12S)
#define LEN_ANTENNA_MODES      "\021"
#define TR_ANTENNA_MODES       "Interne\0         ""Demander\0        ""Par mod\201le\0      ""Interne + Externe"
#else
#define LEN_ANTENNA_MODES      "\012"
#define TR_ANTENNA_MODES       "Interne\0  ""Demander\0 ""Par mod\201le""Externe\0 "
#endif
#define TR_USE_INTERNAL_ANTENNA        TR("Util antenne int", "Utiliser antenne interne")
#define TR_USE_EXTERNAL_ANTENNA        TR("Util antenne ext", "Utiliser antenne externe")
#define TR_ANTENNACONFIRM2             TR("V\200rif antenne", "Installer l'antenne d'abord!")
#define TR_MODULE_PROTOCOL_FLEX_WARN_LINE1        "N\200cessite firm."
#define TR_MODULE_PROTOCOL_FCC_WARN_LINE1         "N\200cessite FCC"
#define TR_MODULE_PROTOCOL_EU_WARN_LINE1          "N\200cessite EU"
#define TR_MODULE_PROTOCOL_WARN_LINE2             "R9M non certifi\200"
#define TR_LOWALARM                    INDENT "Alarme basse"
#define TR_CRITICALALARM               INDENT "Alarme critique"
#define TR_RSSIALARM_WARN              TR("RSSI", "TELEMETRIE")
#define TR_NO_RSSIALARM                TR(INDENT "Alarmes d\200sact.", "Alarme t\200l\200m\200trie d\200sactiv\200e")
#define TR_DISABLE_ALARM               TR(INDENT "D\200sact. alarme", INDENT "D\200sactiver alarme t\200l\200m\200trie")
#define TR_ENABLE_POPUP                "Activer popup"
#define TR_DISABLE_POPUP               "D\200sactiver popup"
#define TR_POPUP                       "Popup"
#define TR_MIN                         "Min"
#define TR_MAX                         "Max"
#define TR_CURVE_PRESET                "Courbe standard..."
#define TR_PRESET                      "Pente"
#define TR_MIRROR                      "Miroir"
#define TR_CLEAR                       "Effacer"
#define TR_RESET                       "R\200initialiser"
#define TR_RESET_SUBMENU               "R\200initialiser..."
#define TR_COUNT                       "Nb points"
#define TR_PT                          "pt"
#define TR_PTS                         "pts"
#define TR_SMOOTH                      "Lissage"
#define TR_COPY_STICKS_TO_OFS          TR("Cpy stick->subtrim", "Manche vers subtrim")
#define TR_COPY_MIN_MAX_TO_OUTPUTS     TR("Cpy min/max to all",  "Copy min/max/center to all outputs")
#define TR_COPY_TRIMS_TO_OFS           TR("Cpy trim->subtrim", "Trim vers subtrim")
#define TR_INCDEC                      "Inc/d\200crementer"
#define TR_GLOBALVAR                   "Var. globale"
#define TR_MIXSOURCE                   "Source mixeur"
#define TR_CONSTANT                    "Constante"
#define TR_PERSISTENT_MAH              TR(INDENT "Enr. mAh", INDENT "Enregistrer mAh")
#define TR_PREFLIGHT                   "V\200rifications avant vol"
#define TR_CHECKLIST                   TR(INDENT "Notes", INDENT "Afficher notes")
#define TR_FAS_OFFSET                  TR(INDENT "Corr FAS", INDENT "Correction FAS")
#define TR_AUX_SERIAL_MODE             "Port s\200rie"
#define TR_AUX2_SERIAL_MODE            "Port s\200rie 2"
#define TR_SCRIPT                      "Script"
#define TR_INPUTS                      "Entr\200es"
#define TR_OUTPUTS                     "Sorties"
#if defined(COLORLCD)
#define STR_EEBACKUP                   "Sauvegarder l'EEPROM"
#define STR_FACTORYRESET               "RAZ d'usine"
#elif defined(PCBXLITE)
#define STR_EEBACKUP                   "Sauvegarde"
#define STR_FACTORYRESET               "RAZ usine"
#else
#define STR_EEBACKUP                   TR("Sauvegarde", "Sauvegarder l'EEPROM")
#define STR_FACTORYRESET               "RAZ d'usine"
#endif
#define TR_CONFIRMRESET                TR("Effacer TOUT?","Effacer TOUS mod\201les/r\200glages?")
#define TR_TOO_MANY_LUA_SCRIPTS         "Trop de scripts lua!"
#define TR_SPORT_UPDATE_POWER_MODE     "SP Power"
#define LEN_SPORT_UPDATE_POWER_MODES   "\004"
#define TR_SPORT_UPDATE_POWER_MODES    "AUTO""ON\0 "
#define TR_NO_TELEMETRY_SCREENS        "No Telemetry Screens"
#define TR_TOUCH_PANEL                 "Touch panel:"

// Horus and Taranis column headers
#define TR_PHASES_HEADERS_NAME         "Nom"
#define TR_PHASES_HEADERS_SW           "Inter"
#define TR_PHASES_HEADERS_RUD_TRIM     "Trim Dir"
#define TR_PHASES_HEADERS_ELE_TRIM     "Trim Prf"
#define TR_PHASES_HEADERS_THT_TRIM     "Trim Gaz"
#define TR_PHASES_HEADERS_AIL_TRIM     "Trim Ail"
#define TR_PHASES_HEADERS_CH5_TRIM     "Trim 5"
#define TR_PHASES_HEADERS_CH6_TRIM     "Trim 6"
#define TR_PHASES_HEADERS_FAD_IN       "Fondu ON"
#define TR_PHASES_HEADERS_FAD_OUT      "Fondu OFF"

#define TR_LIMITS_HEADERS_NAME         "Nom"
#define TR_LIMITS_HEADERS_SUBTRIM      "Subtrim"
#define TR_LIMITS_HEADERS_MIN          "Min"
#define TR_LIMITS_HEADERS_MAX          "Max"
#define TR_LIMITS_HEADERS_DIRECTION    "Direction"
#define TR_LIMITS_HEADERS_CURVE        "Courbe"
#define TR_LIMITS_HEADERS_PPMCENTER    "Neutre PPM"
#define TR_LIMITS_HEADERS_SUBTRIMMODE  "Mode subtrim"

#define TR_LSW_HEADERS_FUNCTION        "Fonction"
#define TR_LSW_HEADERS_V1              "V1"
#define TR_LSW_HEADERS_V2              "V2"
#define TR_LSW_HEADERS_ANDSW           "ET suppl."
#define TR_LSW_HEADERS_DURATION        "Dur\200e"
#define TR_LSW_HEADERS_DELAY           "D\200lai"

#define TR_GVAR_HEADERS_NAME           "Nom"
#define TR_GVAR_HEADERS_FM0            "Valeur FM0"
#define TR_GVAR_HEADERS_FM1            "Valeur FM1"
#define TR_GVAR_HEADERS_FM2            "Valeur FM2"
#define TR_GVAR_HEADERS_FM3            "Valeur FM3"
#define TR_GVAR_HEADERS_FM4            "Valeur FM4"
#define TR_GVAR_HEADERS_FM5            "Valeur FM5"
#define TR_GVAR_HEADERS_FM6            "Valeur FM6"
#define TR_GVAR_HEADERS_FM7            "Valeur FM7"
#define TR_GVAR_HEADERS_FM8            "Valeur FM8"

// Horus footer descriptions
#define TR_LSW_DESCRIPTIONS            { "Type de comparaison ou fonction", "Premi\201re variable", "Seconde variable ou constante", "Seconde variable ou constante", "Condition suppl\200mentaire pour activer la ligne", "Dur\200e minimale d'activation de l'inter logique", "Dur\200e min de la condition avant l'activation de l'inter" }

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

// Taranis About screen
#define TR_ABOUTUS                     TR(" A PROPOS ", "A PROPOS")

#define TR_ABOUT_OPENTX_1              TR("OpenTX\001est\001libre,\001non-", "OpenTX est libre, non-")
#define TR_ABOUT_OPENTX_2              TR("commercial,\001et\001offert\001sans", "commercial et n'offre aucune")
#define TR_ABOUT_OPENTX_3              TR("garantie.\001Il\001est\001d\200velopp\200", "garantie. Il a \200t\200 d\200velopp\200")
#define TR_ABOUT_OPENTX_4              TR("gratuitement.\001Donations", "gratuitement. Vos donations")
#define TR_ABOUT_OPENTX_5              TR("bienvenues!", "sont bienvenues!")

#define TR_ABOUT_BERTRAND_1            "Bertrand Songis"
#define TR_ABOUT_BERTRAND_2            TR("Auteur\001principal\001d'OpenTX", "Auteur principal d'OpenTX")
#define TR_ABOUT_BERTRAND_3            TR("Cod\200veloppeur\001de\001C9X", "Cod\200veloppeur de Companion")

#define TR_ABOUT_MIKE_1                "Mike Blandford"
#define TR_ABOUT_MIKE_2                "Ma\203tre du code et des"
#define TR_ABOUT_MIKE_3                "drivers"
#define TR_ABOUT_MIKE_4                ""

#define TR_ABOUT_ROMOLO_1              "Romolo Manfredini"
#define TR_ABOUT_ROMOLO_2              "D\200veloppeur principal de"
#define TR_ABOUT_ROMOLO_3              "OpenTX Companion"

#define TR_ABOUT_ANDRE_1               "Andr\200 Bernet"
#define TR_ABOUT_ANDRE_2               TR("Fonctions\200,\001usabilit\200,","Fonctionnalit\200s, usabilit\200,")
#define TR_ABOUT_ANDRE_3               TR("d\200bogage,\001documentation","d\200bogage, documentation")

#define TR_ABOUT_ROB_1                 "Rob Thomson"
#define TR_ABOUT_ROB_2                 "Webmaster d'openrcforums"

#define TR_ABOUT_KJELL_1               "Kjell Kernen"
#define TR_ABOUT_KJELL_2               "Auteur de www.open-tx.org"
#define TR_ABOUT_KJELL_3               "Auteur de OpenTX Recorder"
#define TR_ABOUT_KJELL_4               "D\200veloppeur Companion"

#define TR_ABOUT_MARTIN_1              "Martin Hotar"
#define TR_ABOUT_MARTIN_2              "Design graphique"

#if defined(MANUFACTURER_FRSKY)
  #define TR_ABOUT_HARDWARE_1          "FrSky"
  #define TR_ABOUT_HARDWARE_2          "D\200veloppeur/fabricant"
  #define TR_ABOUT_HARDWARE_3          "du mat\200riel"
#elif defined(MANUFACTURER_RADIOMASTER)
  #define TR_ABOUT_HARDWARE_1          "Radiomaster"
  #define TR_ABOUT_HARDWARE_2          "D\200veloppeur/fabricant"
  #define TR_ABOUT_HARDWARE_3          "du mat\200riel"
#elif defined(MANUFACTURER_JUMPER)
  #define TR_ABOUT_HARDWARE_1          "JumperRC"
  #define TR_ABOUT_HARDWARE_2          "D\200veloppeur/fabricant"
  #define TR_ABOUT_HARDWARE_3          "du mat\200riel"
#else
  #define TR_ABOUT_HARDWARE_1          "Brent Nelson"
  #define TR_ABOUT_HARDWARE_2          "D\200veloppeur/fabricant"
  #define TR_ABOUT_HARDWARE_3          "de la carte Sky9x"
#endif

#define TR_ABOUT_PARENTS_1             "Projets parents"
#define TR_ABOUT_PARENTS_2             "ersky9x (Mike Blandford)"
#define TR_ABOUT_PARENTS_3             "ER9X (Erez Raviv)"
#define TR_ABOUT_PARENTS_4             "TH9X (Thomas Husterer)"

#define TR_CHR_SHORT                   's'
#define TR_CHR_LONG                    'l'
#define TR_CHR_TOGGLE                  't'
#define TR_CHR_HOUR                    'h'
#define TR_CHR_INPUT                   'E'   // Values between A-I will work

#define TR_BEEP_VOLUME                 "Volume bips"
#define TR_WAV_VOLUME                  "Volume audio"
#define TR_BG_VOLUME                   "Volume musique"

#define TR_TOP_BAR                     "Barre titre"
#define TR_FLASH_ERASE                 "Flash erase..."
#define TR_FLASH_WRITE                 "Flash write..."
#define TR_OTA_UPDATE                  "OTA update..."
#define TR_MODULE_RESET                "Module reset..."
#define TR_UNKNOWN_RX                  "Unknown RX"
#define TR_UNSUPPORTED_RX              "Unsupported RX"
#define TR_OTA_UPDATE_ERROR            "OTA update error"
#define TR_DEVICE_RESET                "Device reset..."
#define TR_ALTITUDE                    INDENT "Altitude"
#define TR_SCALE                       "Echelle"
#define TR_VIEW_CHANNELS               "Voir voies"
#define TR_VIEW_NOTES                  "Voir notes"
#define TR_MODEL_SELECT                "S\200lection mod\201le"
#define TR_MODS_FORBIDDEN              "Modifications d\200sactiv\200es!"
#define TR_UNLOCKED                    "D\200verrouill\200"
#define TR_ID                          "ID"
#define TR_PRECISION                   "Pr\200cision"
#define TR_RATIO                       "Ratio"
#define TR_FORMULA                     "Formule"
#define TR_CELLINDEX                   "Index \200lem."
#define TR_LOGS                        "Logs"
#define TR_OPTIONS                     "Options"
#define TR_FIRMWARE_OPTIONS            "Firmware options"

#define TR_ALTSENSOR                   "Capteur Alt"
#define TR_CELLSENSOR                  "Capteur El\200m"
#define TR_GPSSENSOR                   "Capteur GPS"
#define TR_CURRENTSENSOR               "Capteur"
#define TR_AUTOOFFSET                  "Offset auto"
#define TR_ONLYPOSITIVE                "Positif"
#define TR_FILTER                      "Filtrage"
#define TR_TELEMETRYFULL               "Plus de capteurs libres!"
#define TR_SERVOS_OK                   "Servos OK"
#define TR_SERVOS_KO                   "Servos KO"
#define TR_INVERTED_SERIAL             INDENT "Invers\200"
#define TR_IGNORE_INSTANCE             TR(INDENT "Ign. inst", INDENT "Ignorer instance")
#define TR_DISCOVER_SENSORS            "D\200couvrir capteurs"
#define TR_STOP_DISCOVER_SENSORS       "Terminer d\200couverte"
#define TR_DELETE_ALL_SENSORS          TR("Suppr. tous capteurs", "Supprimer tous capteurs")
#define TR_CONFIRMDELETE               TR("Tout effacer?", "Vraiment tout " LCDW_128_480_LINEBREAK "effacer ?")
#define TR_SELECT_WIDGET               "S\200lect. widget"
#define TR_REMOVE_WIDGET               "Supprimer widget"
#define TR_WIDGET_SETTINGS             "R\200glages widget"
#define TR_REMOVE_SCREEN               "Supprimer \200cran"
#define TR_SETUP_WIDGETS               "Configurer widgets"
#define TR_USER_INTERFACE              "Interface utilisateur"
#define TR_THEME                       "Th\201me"
#define TR_SETUP                       "Configuration"
#define TR_MAINVIEWX                   "Vue principale X"
#define TR_LAYOUT                      "Disposition"
#define TR_ADDMAINVIEW                 "Ajouter vue principale"
#define TR_BACKGROUND_COLOR            "Couleur de fond"
#define TR_MAIN_COLOR                  "Couleur principale"
#define TR_BAR2_COLOR                  "Secondary bar color"
#define TR_BAR1_COLOR                  "Main bar color"
#define TR_TEXT_COLOR                  "Text color"
#define TR_TEXT_VIEWER                 "Visualisateur de texte"

#define TR_MENU_INPUTS                 "\314Entr\200es"
#define TR_MENU_LUA                    "\322Scripts Lua"
#define TR_MENU_STICKS                 "\307Manches"
#define TR_MENU_POTS                   "\310Pots"
#define TR_MENU_MAX                    "\315MAX"
#define TR_MENU_HELI                   "\316Cyclique"
#define TR_MENU_TRIMS                  "\313Trims"
#define TR_MENU_SWITCHES               "\312Inters"
#define TR_MENU_LOGICAL_SWITCHES       "\312Inters logiques"
#define TR_MENU_TRAINER                "\317Ecolage"
#define TR_MENU_CHANNELS               "\320Canaux"
#define TR_MENU_GVARS                  "\311Vars. glob."
#define TR_MENU_TELEMETRY              "\321T\200l\200m\200trie"
#define TR_MENU_DISPLAY                "AFFICHAGE"
#define TR_MENU_OTHER                  "Autres"
#define TR_MENU_INVERT                 "Inverser"
#define TR_JITTER_FILTER               "Filtre ADC"
#define TR_RTC_CHECK                   TR("V\200rif. RTC", "V\200rif. pile RTC")
#define TR_AUTH_FAILURE                "Auth-failure"
#define TR_RACING_MODE                 "Racing mode"

#define ZSTR_VFR                       "\026\006\022"
#define ZSTR_RSSI                      "\022\023\023\011"
#define ZSTR_R9PW                      "\022\044\020\027"
#define ZSTR_RAS                       "\023\027\022"
#define ZSTR_A1                        "\001\034"
#define ZSTR_A2                        "\001\035"
#define ZSTR_A3                        "\001\036"
#define ZSTR_A4                        "\001\037"
#define ZSTR_BATT                      "\002\354\022\350"
#define ZSTR_ALT                       "\001\364\354"
#define ZSTR_TEMP1                     "\024\363\360\034"
#define ZSTR_TEMP2                     "\024\363\360\035"
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
#define ZSTR_RPM                       "\022\020\015"
#define ZSTR_FUEL                      "\006\353\373\364"
#define ZSTR_VSPD                      "\026\023\360\374"
#define ZSTR_ACCX                      "\001\375\375\030"
#define ZSTR_ACCY                      "\001\375\375\031"
#define ZSTR_ACCZ                      "\001\375\375\032"
#define ZSTR_GYROX                     "\007\031\022\030"
#define ZSTR_GYROY                     "\007\031\022\031"
#define ZSTR_GYROZ                     "\007\031\022\032"
#define ZSTR_CURR                      "\003\353\356\356"
#define ZSTR_CAPACITY                  "\003\377\360\377"
#define ZSTR_VFAS                      "\026\006\001\023"
#define ZSTR_BATT_PERCENT              "\002\377\354%"
#define ZSTR_ASPD                      "\001\023\360\374"
#define ZSTR_GSPD                      "\007\023\360\374"
#define ZSTR_HDG                       "\010\374\371"
#define ZSTR_SATELLITES                "\023\377\354\355"
#define ZSTR_CELLS                     "\003\373\364\355"
#define ZSTR_GPSALT                    "\007\001\364\354"
#define ZSTR_GPSDATETIME               "\004\377\354\373"
#define ZSTR_GPS                       "\007\020\023"
#define ZSTR_BATT1_VOLTAGE             "\022\002\034\026"
#define ZSTR_BATT2_VOLTAGE             "\022\002\035\026"
#define ZSTR_BATT1_CURRENT             "\022\002\034\001"
#define ZSTR_BATT2_CURRENT             "\022\002\035\001"
#define ZSTR_BATT1_CONSUMPTION         "\022\002\034\003"
#define ZSTR_BATT2_CONSUMPTION         "\022\002\035\003"
#define ZSTR_BATT1_TEMP                "\022\002\034\024"
#define ZSTR_BATT2_TEMP                "\022\002\035\024"
#define ZSTR_RB_STATE                  "\022\002\023"
#define ZSTR_CHANS_STATE               "\022\002\003\023"
#define ZSTR_RX_RSSI1                  "\034\022\023\023"
#define ZSTR_RX_RSSI2                  "\035\022\023\023"
#define ZSTR_RX_QUALITY                "\022\021\364\347"
#define ZSTR_RX_SNR                    "\022\023\016\022"
#define ZSTR_RX_NOISE                  "\022\016\355\373"
#define ZSTR_ANTENNA                   "\001\016\024"
#define ZSTR_RF_MODE                   "\022\006\015\004"
#define ZSTR_TX_POWER                  "\024\020\027\022"
#define ZSTR_TX_RSSI                   "\024\022\023\023"
#define ZSTR_TX_QUALITY                "\024\021\364\347"
#define ZSTR_RX_RSSI_PERC              "\022\022\023\020"
#define ZSTR_RX_RF_POWER               "\022\020\027\022"
#define ZSTR_TX_RSSI_PERC              "\024\022\023\020"
#define ZSTR_TX_RF_POWER               "\024\020\027\022"
#define ZSTR_TX_FPS                    "\024\006\020\023"
#define ZSTR_TX_SNR                    "\024\023\016\022"
#define ZSTR_TX_NOISE                  "\024\016\355\373"
#define ZSTR_PITCH                     "\020\354\375\370"
#define ZSTR_ROLL                      "\022\361\364\364"
#define ZSTR_YAW                       "\031\377\351"
#define ZSTR_FLIGHT_MODE               "\020\026"
#define ZSTR_THROTTLE                  "\024\370\356"
#define ZSTR_QOS_A                     "\006\374\373\001"
#define ZSTR_QOS_B                     "\006\374\373\002"
#define ZSTR_QOS_L                     "\006\374\373\014"
#define ZSTR_QOS_R                     "\006\374\373\022"
#define ZSTR_QOS_F                     "\006\014\355\355"
#define ZSTR_QOS_H                     "\010\361\364\374"
#define ZSTR_BIND                      "\002\011\016\004"
#define ZSTR_LAP_NUMBER                "\014\377\360 "
#define ZSTR_GATE_NUMBER               "\007\377\354\373"
#define ZSTR_LAP_TIME                  "\014\377\360\024"
#define ZSTR_GATE_TIME                 "\007\354\373\024"
#define ZSTR_ESC_VOLTAGE               "\005\355\375\026"
#define ZSTR_ESC_CURRENT               "\005\355\375\001"
#define ZSTR_ESC_RPM                   "\005\356\360\363"
#define ZSTR_ESC_CONSUMPTION           "\005\355\375\003"
#define ZSTR_ESC_TEMP                  "\005\355\375\024"
#define ZSTR_SD1_CHANNEL               "\003\370\377\362"
#define ZSTR_GASSUIT_TEMP1             "\007\024\360\034"
#define ZSTR_GASSUIT_TEMP2             "\007\024\360\035"
#define ZSTR_GASSUIT_RPM               "\007\022\020\015"
#define ZSTR_GASSUIT_FLOW              "\007\006\364\361"
#define ZSTR_GASSUIT_CONS              "\007\006\353\373"
#define ZSTR_GASSUIT_RES_VOL           "\007\022\026\364"
#define ZSTR_GASSUIT_RES_PERC          "\007\022\020\375"
#define ZSTR_GASSUIT_MAX_FLOW          "\007\015\006\364"
#define ZSTR_GASSUIT_AVG_FLOW          "\007\001\006\364"
#define ZSTR_SBEC_VOLTAGE              "\002\373\375\026"
#define ZSTR_SBEC_CURRENT              "\002\373\375\001"
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

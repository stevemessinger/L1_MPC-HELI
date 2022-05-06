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

// DE translations author: Helmut Renz
// German checked 28.08.2019 r158 opentx V2.3.0 f\205r X12S,X10,X9E,X9D+,X9D,QX7 X9Lite,XLite
/*
 * !!!!! DO NOT EDIT de.h - EDIT de.h.txt INSTEAD !!!!!!!
 *
 * In order to make translations easier de.h.txt is parsed and national
 * characters are replaced by bitmap codes. The result is de.h.
 *
 * See translate.py in the util folder for the list of character codes
 *
 * !!!!! DO NOT EDIT de.h - EDIT de.h.txt INSTEAD !!!!!!!
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
#define TR_OFFON                       "AUS""EIN"

#define LEN_MMMINV                     "\003"
#define TR_MMMINV                      "---""INV"

#define LEN_VBEEPMODE                  "\005"
#define TR_VBEEPMODE                   "Stumm""Alarm""NoKey""Alle\0"

#define LEN_VBLMODE                    "\005"
#define TR_VBLMODE                     "AUS\0 ""Taste""Stks\0""Beide""EIN\0 " // Anpassung

#define LEN_TRNMODE                    "\003"
#define TR_TRNMODE                     "AUS"" +="" :="

#define LEN_TRNCHN                     "\003"
#define TR_TRNCHN                      "CH1CH2CH3CH4"

#define LEN_AUX_SERIAL_MODES           "\015"
#if defined(CLI) || defined(DEBUG)
#define TR_AUX_SERIAL_MODES    			"Debug\0       ""Telem Mirror\0""Telemetry In\0""SBUS Eingang\0""LUA\0         "
#else
#define TR_AUX_SERIAL_MODES    			"AUS\0         ""Telem Mirror\0""Telemetry In\0""SBUS Eingang\0""LUA\0         "
#endif

#define LEN_SWTYPES            			"\006"
#define TR_SWTYPES            			 "Kein\0 ""Taster""2POS\0 ""3POS\0"

#define LEN_POTTYPES          			 TR("\013","\017")
#define TR_POTTYPES           			 TR("None\0      ""Pot w. det\0""Multipos\0  ""Pot\0       ", "Kein\0          ""Poti mit Raste ""Stufen-Schalter""Poti ohne Raste\0")

#define LEN_SLIDERTYPES        			"\006"
#define TR_SLIDERTYPES         			"Keine\0""Slider"

#define LEN_VLCD               			"\006"
#define TR_VLCD               			 "NormalOptrex"

#define LEN_VPERSISTENT        			"\014"
#define TR_VPERSISTENT         			"AUS\0        ""Flugzeit\0   ""Manuell Ruck"

#define LEN_COUNTRYCODES       			TR("\002", "\007")
#define TR_COUNTRYCODES        			TR("US""JP""EU", "Amerika""Japan\0 ""Europa\0")

#if defined(RADIO_FAMILY_TBS)
#define LEN_USBMODES                   TR("\006", "\010")
#define TR_USBMODES                    TR("Ask\0  ""Joyst\0""Agent\0""SDCard""Serial", "Ask\0    ""Joystick""Agent\0 ""Storage\0""Serial\0 ")
#else
#define LEN_USBMODES           			TR("\006", "\010")
#if defined(DEBUG)
#define TR_USBMODES            			TR("Fragen""Joyst\0""SDCard""Serial", "Fragen\0 ""Joystick""Speicher""Seriell\0")
#else
#define TR_USBMODES            			TR("Fragen""Joyst\0""SDCard""Telem\0", "Fragen\0 ""Joystick""Speicher""Telem\0 ")
#endif
#endif
#define LEN_JACKMODES                  "\007"
#define TR_JACKMODES                   "Popup\0 ""Audio\0 ""Trainer"

#define LEN_TELEMETRY_PROTOCOLS 		"\017"
#define TR_TELEMETRY_PROTOCOLS 			"FrSky S.PORT\0  ""FrSky D\0       ""FrSky D (Kabel)""TBS Crossfire\0 ""Spektrum\0      ""AFHDS2A IBUS\0  ""Multi Telemetry"

#define TR_MULTI_CUSTOM        			"Custom"

#define LEN_VTRIMINC          			 TR("\007", "\014") // urspr\205glich "\006", "\013"
#define TR_VTRIMINC           			 TR("Expo   ""ExFein ""Fein   ""Mittel ""Grob   ", "Exponentiell""Extrafein   ""Fein        ""Mittel      ""Grob        ")

#define LEN_VDISPLAYTRIMS      			"\006"
#define TR_VDISPLAYTRIMS       			"Nein\0 ""Kurz\0 ""Ja\0"  //Trimmwerte Keine, kurze Anzeigen, Ja

#define LEN_VBEEPCOUNTDOWN     			"\006"
#define TR_VBEEPCOUNTDOWN      			"Kein\0 Pieps\0StimmeHaptik"

#define LEN_VVARIOCENTER       			"\006"
#define TR_VVARIOCENTER        			"Ton\0  ""Ruhe  "

#define LEN_CURVE_TYPES       			 "\010"
#define TR_CURVE_TYPES        			 " Nur Y  "" X und Y"      //"Standard""Custom\0"

#define LEN_RETA123           			 "\001"

#if defined(PCBHORUS)
  #define TR_RETA123           			"SHGQ13245LR"
#elif defined(PCBX9E)
  #define TR_RETA123           			"SHGQ1234LRLR"
#elif defined(PCBTARANIS) || defined(REVX)
  #define TR_RETA123          			 "SHGQ123LR"
#elif defined(PCBSKY9X)
  #define TR_RETA123          			 "SHGQ123a"
#else
  #define TR_RETA123          			 "SHGQ123"
#endif

#if defined(PCBSKY9X) && defined(REVX)
  #define LEN_VOUTPUT_TYPE     			"\011"
  #define TR_VOUTPUT_TYPE     			"OpenDrain""PushPull\0"
#endif

#define LEN_VCURVEFUNC                 "\003"
#define TR_VCURVEFUNC                  "---""x>0""x<0""|x|""f>0""f<0""|f|"

#define LEN_VMLTPX                     "\010"
#define TR_VMLTPX                      "Addiere ""Multipl.""Ersetze "

#define LEN_VMLTPX2                    "\002"
#define TR_VMLTPX2                     "+=""*="":="

#define LEN_VMIXTRIMS                  "\003"

#if defined(PCBHORUS)
  #define TR_VMIXTRIMS                 "AUS""EIN""Sei""H\203h""Gas""Que""T5\0""T6\0"
#else
  #define TR_VMIXTRIMS                 "AUS""EIN""Sei""H\203h""Gas""Que"
#endif

#if LCD_W >= 212
  #define TR_CSWTIMER                  "Takt\0" // TIM = Takt = Taktgenerator
  #define TR_CSWSTICKY                 "SRFF\0" // Sticky = RS-Flip-Flop
  #define TR_CSWRANGE                  "Range"  // Range = Bereichsabfrage von bis
  #define TR_CSWSTAY                   "Puls\0" // Edge = einstellbarer Impuls
#else
  #define TR_CSWTIMER                  "Takt\0" // TIM = Takt = Taktgenerator
  #define TR_CSWSTICKY                 "SRFF\0" // Sticky = RS-Flip-Flop
    #define TR_CSWRANGE                "Rnge\0" // Range= Bereichsabfrage von bis
    #define TR_CSWSTAY                 "Puls\0" // Edge = einstellbarer Impuls
#endif

#define TR_CSWEQUAL                    "a=x\0 "

#define LEN_VCSWFUNC                   "\005"
#define TR_VCSWFUNC            			"---\0 " TR_CSWEQUAL "a\173x\0 ""a>x\0 ""a<x\0 " TR_CSWRANGE "|a|>x""|a|<x""AND\0 ""OR\0  ""XOR\0 " TR_CSWSTAY "a=b\0 ""a>b\0 ""a<b\0 ""\306}x\0 ""|\306|}x" TR_CSWTIMER TR_CSWSTICKY

#define LEN_VFSWFUNC                   "\012"

#if defined(VARIO)
  #define TR_VVARIO                    "Vario\0    " // nur 10 Zeichen String!
#else
  #define TR_VVARIO                    "[Vario]\0  "
#endif

#if defined(AUDIO)
  #define TR_SOUND                     "Spiel T\203ne"
#else
  #define TR_SOUND                     "Spiel\0    "
#endif

#if defined(HAPTIC)
  #define TR_HAPTIC                    "Haptik\0   "
#else
  #define TR_HAPTIC                    "[Haptik]\0 "
#endif

#if defined(VOICE)
  #if defined(PCBSKY9X)
    #define TR_PLAY_TRACK              "Spiel\0    "
  #else
    #define TR_PLAY_TRACK              "Sag Text\0 "
  #endif
  #define TR_PLAY_BOTH                 "Sag Beide "
  #define TR_PLAY_VALUE                TR("Sag Wert\0 ", "Sag Wert  ")
#else
  #define TR_PLAY_TRACK                "[Sag Text]"
  #define TR_PLAY_BOTH                 "[SagBeide]"
  #define TR_PLAY_VALUE                "[Sag Wert]"
#endif

#define TR_SF_BG_MUSIC                 "StartMusik""Stop Musik"

#if defined(SDCARD)
  #define TR_SDCLOGS                   "SD-Aufz.\0 "
#else
  #define TR_SDCLOGS                   "[SD Aufz.]"
#endif

#if defined(GVARS)
  #define TR_ADJUST_GVAR               "\200ndere\0   "
#else
  #define TR_ADJUST_GVAR               "[\200ndereGV]"
#endif

#if defined(LUA)
  #define TR_SF_PLAY_SCRIPT             "Lua Skript"
#else
  #define TR_SF_PLAY_SCRIPT             "[Lua]\0    "
#endif

#if defined(DEBUG)
  #define TR_SF_TEST                    "Test\0     "
#else
  #define TR_SF_TEST
#endif

#if defined(OVERRIDE_CHANNEL_FUNCTION) && LCD_W >= 212
  #define TR_SF_SAFETY        			"Override\0 "   //"Override\0 "
#elif defined(OVERRIDE_CHANNEL_FUNCTION)
  #define TR_SF_SAFETY        			"Overr.\0   "   //"Overr.\0   "
#else
  #define TR_SF_SAFETY        			"---\0      "
#endif

#define TR_SF_SCREENSHOT    			"Screenshot"
#define TR_SF_RACING_MODE         "RacingMode"
#define TR_SF_RESERVE        			"[Reserve]\0"

#define TR_VFSWFUNC         			 TR_SF_SAFETY "Lehrer \0  ""Inst. Trim""R\205cksetz.\0""Setze \0   " TR_ADJUST_GVAR "Lautstr.\0 " "SetFailsfe" "RangeCheck" "ModuleBind" TR_SOUND TR_PLAY_TRACK TR_PLAY_VALUE TR_SF_RESERVE TR_SF_PLAY_SCRIPT TR_SF_RESERVE TR_SF_BG_MUSIC TR_VVARIO TR_HAPTIC TR_SDCLOGS "LCD Licht\0" TR_SF_SCREENSHOT TR_SF_RACING_MODE TR_SF_TEST

#define LEN_VFSWRESET         			 TR("\004", "\011")

#define TR_FSW_RESET_TELEM  			 TR("Telm", "Telemetrie")

#if LCD_W >= 212
 #define TR_FSW_RESET_TIMERS   			"Timer 1\0 ""Timer 2\0 ""Timer 3\0 "
#else
 #define TR_FSW_RESET_TIMERS  		    "Tmr1""Tmr2""Tmr3"
#endif

#define TR_VFSWRESET 	      		   TR(TR_FSW_RESET_TIMERS "All\0" TR_FSW_RESET_TELEM, TR_FSW_RESET_TIMERS "All\0 " TR_FSW_RESET_TELEM)

#define LEN_FUNCSOUNDS        		   TR("\004", "\006")
#define TR_FUNCSOUNDS         		   TR("Bp1\0""Bp2\0""Bp3\0""Wrn1""Wrn2""Chee""Rata""Tick""Sirn""Ring""SciF""Robt""Chrp""Tada""Crck""Alrm", "Piep1\0""Piep2\0""Piep3\0""Warn1 ""Warn2 ""Cheep ""Ratata""Tick  ""Siren ""Ring  ""SciFi ""Robot ""Chirp ""Tada  ""Crickt""AlmClk")

#define LEN_VTELEMCHNS        		   "\004"

#define LENGTH_UNIT_IMP                "ft\0"
#define SPEED_UNIT_IMP                 "mph"
#define LENGTH_UNIT_METR               "m\0 "
#define SPEED_UNIT_METR                "kmh"

  #define LEN_VUNITSSYSTEM             TR("\006", "\012")
  #define TR_VUNITSSYSTEM              TR("Metrik""Imper.", "Metrisch\0 ""Imperial\0 ")
  #define LEN_VTELEMUNIT               "\003"
  #define TR_VTELEMUNIT                    "-\0 ""V\0 ""A\0 ""mA\0""kts""m/s""f/s""kmh""mph""m\0 ""ft\0""@C\0""@F\0""%\0 ""mAh""W\0 ""mW\0""dB\0""rpm""g\0 ""@\0 ""rad""ml\0""fOz""mlm""Hz\0""mS\0""uS\0""km\0""dbm"

#define STR_V                  			(STR_VTELEMUNIT+1)
#define STR_A                 			 (STR_VTELEMUNIT+4)

#define LEN_VTELEMSCREENTYPE 			"\007"
#define TR_VTELEMSCREENTYPE  			" None  "" Werte "" Balken"" Script"

#define LEN_GPSFORMAT          			"\004"
#define TR_GPSFORMAT          			 "GMS\0""NMEA"  //Koordinatenanzeige

#define LEN2_VTEMPLATES        			15  // max String L\201nge f\205r Men\205 (original=13)
#define LEN_VTEMPLATES        	       "\017"  // max String L\201nge 15+cr+lf
#define TR_TEMPLATE_CLEAR_MIXES        "Misch. L\203sch.\0 "
#define TR_TEMPLATE_SIMPLE_4CH         "Einfach. 4-CH\0 "
#define TR_TEMPLATE_STICKY_TCUT        "Fixe Gassperre\0"
#define TR_TEMPLATE_VTAIL              "V-Leitwerk\0    "
#define TR_TEMPLATE_DELTA              "Delta Mischer\0 "
#define TR_TEMPLATE_ECCPM              "eCCPM\0         "
#define TR_TEMPLATE_HELI               "Hubschrauber\0  "
#define TR_TEMPLATE_SERVO_TEST         "Servo Tester\0  "

#define LEN_VSWASHTYPE                 "\004"
#define TR_VSWASHTYPE                  "--- ""120 ""120X""140 ""90\0"

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

#define LEN_VSWITCHES                  "\003"
#define LEN_VSRCRAW                    "\004"

#define TR_STICKS_VSRCRAW              "\307Sei""\307H\203h""\307Gas""\307Que"

#if defined(PCBHORUS)
  #define TR_TRIMS_VSRCRAW             "\313Sei""\313H\203h""\313Gas""\313Que""\313T5\0""\313T6\0"
#else
  #define TR_TRIMS_VSRCRAW             TR("TrmS""TrmH""TrmG""TrmQ", "\313Sei""\313H\203h""\313Gas""\313que")
#endif

#if defined(PCBHORUS)
  #define TR_TRIMS_SWITCHES    			"\313Rl""\313Rr""\313Ed""\313Eu""\313Td""\313Tu""\313Al""\313Ar""\3135d""\3135u""\3136d""\3136u"
#else
  #define TR_TRIMS_SWITCHES    			TR("tRl""tRr""tEd""tEu""tTd""tTu""tAl""tAr", "\313Rl""\313Rr""\313Ed""\313Eu""\313Td""\313Tu""\313Al""\313Ar")
#endif

#if defined(PCBSKY9X)
  #define TR_ROTARY_ENCODERS           "DGa\0"
  #define TR_ROTENC_SWITCHES           "DGa"
#else
  #define TR_ROTARY_ENCODERS
  #define TR_ROTENC_SWITCHES
#endif

#define TR_ON_ONE_SWITCHES             "ON\0""One"

#if defined(GYRO)
  #define TR_GYR_VSRCRAW               "GyrX""GyrY"
#else
  #define TR_GYR_VSRCRAW
#endif

#if defined(HELI)
#define TR_CYC_VSRCRAW                 "CYC1""CYC2""CYC3"
#else
#define TR_CYC_VSRCRAW                 "[C1]""[C2]""[C3]"
#endif

#define TR_RESERVE_VSRCRAW   			"[--]"
#define TR_EXTRA_VSRCRAW     			"Batt""Time""GPS\0" TR_RESERVE_VSRCRAW TR_RESERVE_VSRCRAW TR_RESERVE_VSRCRAW TR_RESERVE_VSRCRAW "Tmr1""Tmr2""Tmr3"

#define LEN_VTMRMODES         			 "\003"
#define TR_VTMRMODES          			 "AUS""EIN""GSs""GS%""GSt"

#define LEN_VTRAINERMODES              "\022"
#define TR_VTRAINER_MASTER_JACK        "Lehrer/Buchse\0    "
#define TR_VTRAINER_SLAVE_JACK         "Sch\205ler/Buchse\0   "
#define TR_VTRAINER_MASTER_SBUS_MODULE "Lehrer/SBUS Modul\0"
#define TR_VTRAINER_MASTER_CPPM_MODULE "Lehrer/CPPM Modul\0"
#define TR_VTRAINER_MASTER_BATTERY     "Lehrer/Serial\0    "
#define TR_VTRAINER_BLUETOOTH          TR("Master/BT\0        ""Slave/BT\0         ", "Master/Bluetooth\0 ""Slave/Bluetooth\0  ")
#define TR_VTRAINER_MULTI              "Master/Multi\0     "
#define TR_VTRAINER_SPORT_SBUS         "Master/SBUS-Sport\0"

#define LEN_VFAILSAFE         			 "\015" // "\013" original
#define TR_VFAILSAFE          			 "Kein Failsafe""Halte Pos.\0  ""Kan\201le\0      ""Kein Signal\0 ""Empf\201nger\0   "


#define LEN_VSENSORTYPES      			  "\012"
#define TR_VSENSORTYPES      			  "Sensor\0   ""Berechnung"

#define LEN_VFORMULAS        			  "\014"  // "\10" urspr\205nglich
#define TR_VFORMULAS         			  "Addieren\0   ""Mittelwert\0 ""Min\0        ""Max\0        ""Multiplizier""Gesamt\0     ""Zelle\0      ""Verbrauch\0  ""Distanz\0    "

#define LEN_VPREC           			   "\004"  //  "\005"  Prec0 Prec1 Prec2 urspr\205nglich
#define TR_VPREC            			   "0.--""0.0 ""0.00"

#define LEN_VCELLINDEX      			   "\012"  // "\007" urspr\205nglich
#define TR_VCELLINDEX        			  "Niedrigst\0""1. Zelle\0 ""2. Zelle\0 ""3. Zelle\0 ""4. Zelle\0 ""5. Zelle\0 ""6. Zelle\0 ""H\203chster\0 ""Differenz\0"

#define LEN_GYROS             	         "\004"
#define TR_GYROS              	         "GyrX""GyrY"

#define LEN_TEXT_SIZE                  "\010"
#define TR_TEXT_SIZE                   "Standard""Tiny\0   ""Small\0  ""Mid\0    ""Double\0 "

// ZERO TERMINATED STRINGS
#if defined(COLORLCD)
  #define INDENT              			 "   "
  #define LEN_INDENT         			 3
  #define INDENT_WIDTH        			 12
  #define BREAKSPACE          			 "\036"
#else
  #define INDENT              			 "\001"
  #define LEN_INDENT          			 1
  #define INDENT_WIDTH       			  (FW/2)
  #define BREAKSPACE         			  " "
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

#define TR_FREE                        "frei"
#define TR_DELETEMODEL                 "Modell l\203schen?"
#define TR_COPYINGMODEL                "Kopiere Modell"
#define TR_MOVINGMODEL                 "Schiebe Modell"
#define TR_LOADINGMODEL                "Lade Modell..."
#define TR_NAME                        "Name"
#define TR_MODELNAME                   "Modellname"
#define TR_PHASENAME                   "Phase-Name"
#define TR_MIXNAME                     "Mix-Name"
#define TR_INPUTNAME                   TR("Input", "Input name")
#define TR_EXPONAME                    TR("Name", "Line name")
#define TR_BITMAP                      "Modellfoto"
#define TR_TIMER                       TR("Timer", "Timer ")  // f\205r Timer1 Timer2 Timer3
#define TR_ELIMITS                     TR("Erw. Limit", "Erw. Wege auf 150%")
#define TR_ETRIMS                      TR("Erw. Trims", "Erw. Trim  auf 100%")
#define TR_TRIMINC                     TR("Trimschritt", "Trimmschritte")
#define TR_DISPLAY_TRIMS               TR("Trimanzeige", "Trimwerte anzeigen")
#define TR_TTRACE                      TR("Gasquelle", INDENT "Gas-Timerquelle")
#define TR_TTRIM 	       	           TR("Gastrim", INDENT "Gas-Leerlauftrim")
#define TR_TTRIM_SW                    TR("T-Trim-Sw", INDENT "Trim switch")
#define TR_BEEPCTR                     TR("MittePieps", "Mittelstell. -Pieps")
#define TR_USE_GLOBAL_FUNCS            TR("Glob. Funkt.", "Globale Funkt verw.")
#if defined(PCBSKY9X) && defined(REVX)
  #define TR_OUTPUT_TYPE       		   INDENT "Output"
#endif
#define TR_PROTOCOL          		   TR("Protok.", "Protokoll")
#define TR_PPMFRAME          	  	   INDENT "PPM-Frame"
#define TR_REFRESHRATE             	   TR(INDENT "Refresh", INDENT "Refresh rate")
#define STR_WARN_BATTVOLTAGE           TR(INDENT "Output is VBAT: ", INDENT "Warning: output level is VBAT: ")
#define TR_WARN_5VOLTS                 "Warning: output level is 5 volts"
#define TR_MS                 		   "ms"
#define TR_FREQUENCY                   INDENT "Frequency"
#define TR_SWITCH                      TR("Schalt.", "Schalter")
#define TR_TRIMS                       "Trims"
#define TR_FADEIN                      "Langs. Ein"
#define TR_FADEOUT                     "Langs. Aus"
#define TR_DEFAULT                     "(Normal)"
#define TR_CHECKTRIMS                  CENTER"\006Check\012Trims"
#define OFS_CHECKTRIMS                 CENTER_OFS+(9*FW)
#define TR_SWASHTYPE                   TR("Typ Taumelsch", "Typ  Taumelscheibe")
#define TR_COLLECTIVE                  TR("Kollekt. Pitch", "Kollekt. Pitch Quelle")
#define TR_AILERON                     "Roll Quelle"
#define TR_ELEVATOR                    "Nick Quelle"
#define TR_SWASHRING                   TR("Ring   Begrenz", "Ring Begrenzung")
#define TR_ELEDIRECTION                TR("Nick   Richtung", "Nick   Servo Richtung")
#define TR_AILDIRECTION                TR("Roll   Richtung", "Roll   Servo Richtung")
#define TR_COLDIRECTION                TR("Pitch Richtung", "Pitch Servo Richtung")
#define TR_MODE                        "Modus"
#define TR_SUBTYPE                     INDENT "Subtype"
#define TR_NOFREEEXPO                  "Expos voll!"
#define TR_NOFREEMIXER                 "Mischer voll!"
#define TR_SOURCE                       "Quelle"
#define TR_WEIGHT                      "Gewicht"
#define TR_EXPO                        TR("Expo", "Exponential")
#define TR_SIDE                        "Seite"
#define TR_DIFFERENTIAL                "Diff"
#define TR_OFFSET                       "Offset"
#define TR_TRIM                        "Trim"
#define TR_DREX                        "DRex"
#define DREX_CHBOX_OFFSET              30
#define TR_CURVE                       "Kurve"
#define TR_FLMODE                      TR("Phase", "Phasen")
#define TR_MIXWARNING                  "Warnung"
#define TR_OFF                         "AUS"
#define TR_ANTENNA                     "Antenna"
#define TR_NO_INFORMATION              TR("No info", "No information")
#define TR_MULTPX                      "Wirkung"
#define TR_DELAYDOWN                   "Verz. Dn"
#define TR_DELAYUP                     "Verz. Up"
#define TR_SLOWDOWN                    "Langs.Dn"
#define TR_SLOWUP                      "Langs.Up"
#define TR_MIXES                       "MISCHER"
#define TR_CV                          "KV"
#define TR_GV                          TR("G", "GV")
#define TR_ACHANNEL                    TR("A\004gemessen", "A\004Kanal gemessen =>") //9XR-Pro
#define TR_RANGE                       TR(INDENT "Bereich", INDENT "Variobereich m/s") // 9XR-Pro
#define TR_CENTER                      TR(INDENT "Mitte", INDENT "Variomitte     m/s")
#define TR_BAR                         "Balken"
#define TR_ALARM                       TR(INDENT "Alarme ", INDENT "Aufleuchten bei Alarm")  //9XR-Pro
#define TR_USRDATA                     "Daten berechnen aus"
#define TR_BLADES                      TR(INDENT "Prop", INDENT "Prop-Bl\201tter") //9XR-Pro
#define TR_SCREEN                      "Seite: "
#define TR_SOUND_LABEL                 "T\203ne"
#define TR_LENGTH                      INDENT "Dauer"
#define TR_BEEP_LENGTH                 INDENT "Beep-L\201nge"
#define TR_SPKRPITCH                   INDENT "Beep-Freq +/-"
#define TR_HAPTIC_LABEL                "Haptik"
#define TR_HAPTICSTRENGTH              INDENT "St\201rke"
#define TR_GYRO_LABEL                  "Gyro"
#define TR_GYRO_OFFSET                 "Offset"
#define TR_GYRO_MAX                    "Max"
#define TR_CONTRAST                    "LCD-Kontrast"
#define TR_ALARMS_LABEL                "Alarme"
#define TR_BATTERY_RANGE               TR("Akku Bereich", "Akku Spannungsbereich") // Symbol Akku Ladezustand
#define TR_BATTERYWARNING              TR(INDENT "Akku Warnung", "Akkuspannungswarnung")
#define TR_INACTIVITYALARM             TR(INDENT "Inaktivit\201t", INDENT "Inaktivit\201t nach") //9XR-Pro
#define TR_MEMORYWARNING               INDENT "Speicher voll"
#define TR_ALARMWARNING                TR(INDENT "Alle T\203ne aus?", INDENT "Alle T\203ne ganz aus?")
#define TR_RSSISHUTDOWNALARM           TR(INDENT "RSSI-Chk  aus?", INDENT "Pr\205fe RSSI bei Ausschalten")
#define TR_MODEL_STILL_POWERED         "Model still powered"
#define TR_MODEL_SHUTDOWN              "Shutdown ?"
#define TR_PRESS_ENTER_TO_CONFIRM      "Press enter to confirm"
#define TR_THROTTLE_LABEL              "Gas-Kontrolle"
#define TR_THROTTLEREVERSE             TR("Gas invers", INDENT "Vollgas hinten?") //\200nderung wg TH9x, Taranis
#define TR_MINUTEBEEP                  TR("Min-Alarm", "Minuten-Alarm")
#define TR_BEEPCOUNTDOWN               INDENT "Countdown"
#define TR_PERSISTENT                  TR(INDENT "Permanent", INDENT "Permanent")
#define TR_BACKLIGHT_LABEL             "LCD-Beleuchtung"
#define TR_GHOST_MENU_LABEL            "GHOST MENU"
#define TR_STATUS                      "Status"
#define TR_RAW_12BITS                  "Raw 12 bits"
#define TR_BLDELAY                     INDENT "Dauer"
#define TR_BLONBRIGHTNESS              INDENT "An-Helligkeit"
#define TR_BLOFFBRIGHTNESS             INDENT "Aus-Helligkeit"
#define TR_KEYS_BACKLIGHT              "Keys backlight"
#define TR_BLCOLOR                     INDENT "Farbe"
#define TR_SPLASHSCREEN                TR("Startbild Ein", "Startbild Anzeigedauer")
#define TR_PWR_ON_DELAY                "Pwr On delay"
#define TR_PWR_OFF_DELAY               "Pwr Off delay"
#define TR_THROTTLEWARNING             TR("Gasalarm", INDENT "Gas Alarm")
#define TR_SWITCHWARNING               TR("Sch. Alarm", INDENT "Schalter-Alarm")
#define TR_POTWARNINGSTATE             TR(INDENT "Pot&Slid.", INDENT "Pots & sliders")
#define TR_SLIDERWARNING               TR(INDENT "Slid. pos.", INDENT "Slider positions")
#define TR_POTWARNING                  TR("Potiwarnung", INDENT "Poti-Warnung")
#define TR_TIMEZONE                    TR("Zeitzone", "GPS-Zeitzone +/-Std")
#define TR_ADJUST_RTC                  TR("GPSzeit setzen", INDENT "Uhrzeit per GPS setzen")
#define TR_GPS                         "GPS"
#define TR_RXCHANNELORD                TR("Kanal CH1-4", "Kanalvoreinstellung")
#define TR_STICKS                      "Kn\205ppel"
#define TR_POTS                        "Potis"
#define TR_SWITCHES                    "Schalter"
#define TR_SWITCHES_DELAY      			TR("Sw. Mitte Delay", "Schaltermitte Verz\203gerung")   //Schalter Mitten verz\203gern Anpassung
#define TR_SLAVE                       TR("Sch\205ler PPM1-16", "Sch\205ler PPM1-16 als Ausgang")
#define TR_MODESRC                     " Modus\003%  Quelle"
#define TR_MULTIPLIER                  "Multiplik."
#define TR_CAL                         "Kal."
#define TR_VTRIM                       "Trim - +"
#define TR_BG                          "BG:"
#if defined(PCBHORUS)
  #define TR_MENUTOSTART               "Dr\205cke [Enter] zum Start"
  #define TR_SETMIDPOINT               "Kn\205ppel/Potis/Sliders auf Mitte [Enter]"
  #define TR_MOVESTICKSPOTS            "Kn\205ppel/Potis/Sliders bewegen [Enter]"
#elif defined(COLORLCD)
  #define TR_MENUTOSTART               TR_ENTER " Zum Start"
  #define TR_SETMIDPOINT               "STICKS/SLIDERS/POTIS auf Mitte"
  #define TR_MOVESTICKSPOTS            "STICKS/SLIDERS/POTIS" bewegen
  #define TR_MENUWHENDONE              TR_ENTER " wenn fertig"
#else
  #define TR_MENUTOSTART               CENTER "\010" TR_ENTER " Zum START"
  #define TR_SETMIDPOINT               TR(CENTER "\004Mitte Kn\205ppel/Sliders", CENTER "\004Mitte Kn\205ppel/Potis")
  #define TR_MOVESTICKSPOTS            CENTER "\004Bewege Kn\205ppel/POTIS"
  #define TR_MENUWHENDONE              CENTER "\006" TR_ENTER " wenn fertig"
#endif
#define TR_RXBATT                      "Rx Akku:"
#define TR_TXnRX                       "Tx:\0Rx:"
#define OFS_RX                         4
#define TR_ACCEL                       "Acc:"
#define TR_NODATA                      CENTER"Keine Daten"
#define TR_US                                 "us"
#define TR_TMIXMAXMS         	       "Tmix max"
#define TR_FREE_STACK     		       "Freier Stack"
#define TR_MENUTORESET                 TR_ENTER " f\205r Reset"
#define TR_PPM_TRAINER                 "TR"
#define TR_CH                          "CH"
#define TR_MODEL                       "MODELL"
#define TR_FM                          "FP"
#define TR_MIX                         "MIX"
#define TR_EEPROMLOWMEM                "EEPROM voll"
#define TR_ALERT                       "WARNUNG"
#define TR_PRESSANYKEYTOSKIP	       TR("Taste dr\205cken",CENTER"Taste dr\205cken")
#define TR_THROTTLENOTIDLE             "Gas nicht Null!"
#define TR_ALARMSDISABLED              "Alarme ausgeschaltet"
#define TR_PRESSANYKEY                 TR("Taste dr\205cken",CENTER"Taste dr\205cken")
#define TR_BADEEPROMDATA               "EEPROM ung\205ltig"
#define TR_BAD_RADIO_DATA              "Bad Radio Data"
#define TR_EEPROMFORMATTING            "EEPROM Initialisieren"
#define TR_STORAGE_FORMAT              "Speicher Vorbereiten" //
#define TR_EEPROMOVERFLOW              "EEPROM \204berlauf"
#define TR_MENURADIOSETUP              TR("SENDER-EINSTELLEN", "SENDER-GRUNDEINSTELLUNGEN")
#define TR_MENUDATEANDTIME             "DATUM UND ZEIT"
#define TR_MENUTRAINER                 TR("LEHRER/SCH\204LER", "LEHRER/SCH\204LER")
#define TR_MENUSPECIALFUNCS            "GLOBALE FUNKTIONEN"
#define TR_MENUVERSION                 "VERSION"
#define TR_MENU_RADIO_SWITCHES         TR("Schalter-Test", "Schalter-Test")
#define TR_MENU_RADIO_ANALOGS          "Geber-Test"
#define TR_MENUCALIBRATION             TR("KALIB. ANALOG", "KALIBRIERUNG-Analog")
#if defined(COLORLCD)
  #define TR_TRIMS2OFFSETS             "Trims => Subtrims"
#else
  #define TR_TRIMS2OFFSETS             "\006Trims => Subtrims"
#endif
#define TR_CHANNELS2FAILSAFE           "Channels=>Failsafe"
#define TR_CHANNEL2FAILSAFE            "Channel=>Failsafe"
#define TR_MENUMODELSEL        		   TR("MODELLE", "MODELL W\200HLEN")
#define TR_MENUSETUP          		   TR("MODELL-EINSTELLUNG", "MODELL-EINSTELLUNGEN")
#define TR_MENUFLIGHTMODE    		   "FLUGPHASE"
#define TR_MENUFLIGHTMODES   		   "FLUGPHASEN"
#define TR_MENUHELISETUP               TR("HELI TS-Mischer", "HELI TS-Mischer CYC1-3")

  #define TR_MENUINPUTS                "INPUTS"  //"Inputs=Geber"
  #define TR_MENULIMITS                "SERVOS"  //"AUSGABEN" oder "Servos"
#define TR_MENUCURVES                  "KURVEN"
#define TR_MENUCURVE                   "KURVE"
#define TR_MENULOGICALSWITCH           "LOGIKSCHALTER"
#define TR_MENULOGICALSWITCHES         "LOGIKSCHALTER"
#define TR_MENUCUSTOMFUNC              TR("SPEZ.-FUNKTIONEN", "SPEZIAL-FUNKTIONEN")
#define TR_MENUCUSTOMSCRIPTS           "LUA-SCRIPTE"
#define TR_MENUTELEMETRY               "TELEMETRIE"
#define TR_MENUTEMPLATES               "VORLAGEN"
#define TR_MENUSTAT                    "STAT"
#define TR_MENUDEBUG                   "DEBUG"
#define TR_MONITOR_CHANNELS1           "KANAL+MISCHER MONITOR 1-8"
#define TR_MONITOR_CHANNELS2           "KANAL+MISCHER MONITOR 9-16"
#define TR_MONITOR_CHANNELS3           "KANAL+MISCHER MONITOR 17-24"
#define TR_MONITOR_CHANNELS4           "KANAL+MISCHER MONITOR 25-32"
#define TR_MONITOR_SWITCHES            "LOGIK SCHALTER MONITOR"
#define TR_MONITOR_OUTPUT_DESC         "Kan\201le"
#define TR_MONITOR_MIXER_DESC          "Mischer"
  #define TR_RECEIVER_NUM                TR("Empf Nr.", "Empf\201nger Nummer")
  #define TR_RECEIVER                    "Empf\201nger"
#define TR_MULTI_RFTUNE        			TR(INDENT "RF Freq.", INDENT "RF Freq. Feintuning")
#define TR_MULTI_RFPOWER               "RF power"
#define TR_MULTI_WBUS                  "Output"
#define TR_MULTI_TELEMETRY             "Telemetry"
#define TR_MULTI_VIDFREQ       			TR(INDENT "Vid. Freq.", INDENT "Video Frequenz")
#define TR_RFPOWER              		"RF Power"
#define TR_MULTI_FIXEDID               TR("FixedID", "Fixed ID")
#define TR_MULTI_OPTION        			TR(INDENT "Option", INDENT "Optionswert")
#define TR_MULTI_AUTOBIND      			TR(INDENT "Bind Ch.",INDENT "Bind on channel")
#define TR_DISABLE_CH_MAP              TR("No Ch. map", "Disable Ch. map")
#define TR_DISABLE_TELEM               TR("No Telem", "Disable Telemetry")
#define TR_MULTI_DSM_AUTODTECT 			TR(INDENT "Autodetect", INDENT "Autodetect format")
#define TR_MULTI_LOWPOWER      			TR(INDENT "Low power", INDENT "Low power mode")
#define TR_MULTI_LNA_DISABLE            INDENT "LNA disable"
#define TR_MODULE_TELEMETRY            TR(INDENT "S.Port", INDENT "S.Port link")
#define TR_MODULE_TELEM_ON             TR("ON", "Enabled")
#define TR_DISABLE_INTERNAL            TR("Disable int. RF", "Disable internal RF")
#define TR_MODULE_NO_SERIAL_MODE       TR("!serial mode", "Not in serial mode")
#define TR_MODULE_NO_INPUT             TR("No input", "No serial input")
#define TR_MODULE_NO_TELEMETRY         TR3( "No telmetry", "No MULTI_TELEMETRY", "No telemetry (enable MULTI_TELEMETRY)")
#define TR_MODULE_WAITFORBIND          "Bind to load protocol"
#define TR_MODULE_BINDING              "Binding"
#define TR_MODULE_UPGRADE_ALERT        TR3("Upg. needed", "Module upgrade required", "Modul\036Update n\203tig")
#define TR_MODULE_UPGRADE              TR("Upg. advised", "Modulupdate empfohlen")
#define TR_PULSE_RATE                  "Pulse rate"
#define TR_LINK_SPEED                  "Link speed"
#define TR_REBIND                      "Rebinding required"
#define TR_REG_OK                      "Registration ok"
#define TR_BIND_OK                     "Bind successful"
#define TR_BINDING_CH1_8_TELEM_ON      "Ch1-8 Telem AN"
#define TR_BINDING_CH1_8_TELEM_OFF     "Ch1-8 Telem AUS"
#define TR_BINDING_CH9_16_TELEM_ON     "Ch9-16 Telem AN"
#define TR_BINDING_CH9_16_TELEM_OFF     "Ch9-16 Telem AUS"
#define TR_PROTOCOL_INVALID            TR("Prot. invalid", "Protocol invalid")
#define TR_MODULE_STATUS                TR(INDENT "Status", INDENT "Module Status")
#define TR_MODULE_SYNC                 TR(INDENT "Sync", INDENT "Proto Sync Status")
#define TR_MULTI_SERVOFREQ              TR(INDENT "Servo rate", INDENT "Servo update rate")
#define TR_MULTI_MAX_THROW             TR("Max. Throw", "Enable max. throw")
#define TR_MULTI_RFCHAN                TR("RF Channel", "Select RF channel")
#define TR_SYNCMENU                     "Sync [MENU]"
#define TR_LIMIT                        INDENT "Grenzen"
#define TR_MINRSSI                      "Min. RSSI"
#define TR_LATITUDE                     "Breite:"
#define TR_LONGITUDE                    "L\201nge:"
#define TR_GPSCOORD                     TR("GPS-Koord.", "GPS-Koordinaten-Format")
#define TR_VARIO                        "Variometer"
#define TR_PITCH_AT_ZERO                INDENT "T\203ne sinken"
#define TR_PITCH_AT_MAX                 INDENT "T\203ne steigen"
#define TR_REPEAT_AT_ZERO               INDENT "Wiederholrate"
#define TR_SHUTDOWN                     "Herunterfahren"
#define TR_SAVEMODEL                    "Modelleinstellungen speichern"
#define TR_BATT_CALIB                   TR("AkkuSpgwert", "Akku Kalibrierung")
#define TR_CURRENT_CALIB                "Strom abgl."
#define TR_VOLTAGE                      TR(INDENT "Spg", INDENT "Spannungsquelle")  //9XR-Pro
#define TR_CURRENT                      TR(INDENT "Strom", INDENT "Stromquelle")
#define TR_SELECT_MODEL                 "Modell ausw\201hlen"
#define TR_SELECT_MODE                 "Select mode"
#define TR_CREATE_CATEGORY              "Modelltyp erstellen"
#define TR_RENAME_CATEGORY              "Modelltyp umbenennen"
#define TR_DELETE_CATEGORY              "Modelltyp l\203schen"
#define TR_CREATE_MODEL                 TR("Neues Modell" , "Neues Modell erstellen")
#define TR_DUPLICATE_MODEL              "Kopiere Modell"
#define TR_COPY_MODEL                   "Kopiere Modell"
#define TR_MOVE_MODEL                   "Verschiebe Modell"
#define TR_BACKUP_MODEL                 "Modell auf SD-Karte"  //9XR-Pro
#define TR_DELETE_MODEL                 "L\203sche Modell" // TODO merged into DELETEMODEL?
#define TR_RESTORE_MODEL                TR("Modell wiederher.", "Modell wiederherstellen")
#define TR_DELETE_ERROR                 "Fehler beim\036L\203schen"
#define TR_CAT_NOT_EMPTY                "Modelltyp nicht leer"
#define TR_SDCARD_ERROR                 "SD-Kartenfehler"
#define TR_NO_SDCARD                    "Keine SD-Karte"
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
#define TR_16CH_WITHOUT_TELEMETRY      TR("16CH ohne Telem.", "16CH ohne Telemetry")
#define TR_16CH_WITH_TELEMETRY         TR("16CH mit Telem.", "16CH mit Telemetry")
#define TR_8CH_WITH_TELEMETRY          TR("8CH mit Telem.", "8CH mit Telemetry")
#define TR_EXT_ANTENNA                 "Ext. Antenne"
#define TR_PIN                         "Pin"
#define TR_UPDATE_RX_OPTIONS           "Update RX options?"
#define TR_UPDATE_TX_OPTIONS           "Update TX options?"
#define TR_MODULES_RX_VERSION          "Module / RX version"
#define TR_MENU_MODULES_RX_VERSION     "MODULE / RX VERSION"
#define TR_MENU_FIRM_OPTIONS           "FIRMWARE OPTIONEN"
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
#define TR_SDCARD_FULL         		   "SD-Karte voll"
#define TR_NEEDS_FILE                  "NEEDS FILE"
#define TR_EXT_MULTI_SPEC              "opentx-inv"
#define TR_INT_MULTI_SPEC              "stm-opentx-noinv"
#define TR_INCOMPATIBLE        		    "Nicht kompatibel"
#define TR_WARNING            			"WARNUNG"
#define TR_EEPROMWARN          			"EEPROM"
#define TR_STORAGE_WARNING     			"SPEICHER"
#define TR_EEPROM_CONVERTING   			"EEPROM Konvertierung"
#define TR_THROTTLEWARN        			"GAS"
#define TR_ALARMSWARN          			"ALARM"
#define TR_SWITCHWARN          			"SCHALTER"
#define TR_FAILSAFEWARN        			"FAILSAFE"
#define TR_TEST_WARNING     			TR("TESTING", "TEST BUILD")
#define TR_TEST_NOTSAFE     			"Use for tests only"
#define TR_WRONG_SDCARDVERSION          TR("Erw. Version: ","Erwartete Version: ")
#define TR_WARN_RTC_BATTERY_LOW         "RTC Battery low"
#define TR_WARN_MULTI_LOWPOWER         "Low power mode"
#define TR_BATTERY                      "BATTERY"
#define TR_WRONG_PCBREV                 "Falsche PCB erkannt"
#define TR_EMERGENCY_MODE               "NOTFALL MODUS"
#define TR_PCBREV_ERROR                 "PCB Problem"
#define TR_NO_FAILSAFE                  TR("Failsafe not set", "Failsafe nicht programmiert")
#define TR_KEYSTUCK                     "Taste klemmt"  //Key stuck=Taste klemmt
#define TR_INVERT_THR                   TR("Gas umkehren?", "Vollgas hinten?") // Th9x 9XR
#define TR_SPEAKER_VOLUME               INDENT "Lautst\201rke"
#define TR_LCD                          "Bildschirm"
#define TR_BRIGHTNESS                   INDENT "Helligkeit"
#define TR_CPU_TEMP                     "CPU-Temp.\016>"
#define TR_CPU_CURRENT                  "Strom\022>"
#define TR_CPU_MAH                      "Verbrauch"
#define TR_COPROC                       "CoProz."
#define TR_COPROC_TEMP                  "MB Temp. \016>"
#define TR_CAPAWARNING                  INDENT "Kapaz. niedrig" // wg 9XR-Pro
#define TR_TEMPWARNING                  INDENT "Temp.   gr\203\206er"  //wg 9XR-Pro
#define TR_FUNC                         "Funktion"
#define TR_V1                           "V1"
#define TR_V2                           "V2"
#define TR_DURATION                     "Dauer"
#define TR_DELAY                        "Verz\203g."
#define TR_SD_CARD                      "SD-Karte"
#define TR_SDHC_CARD                    "SDHC-Karte"
#define TR_NO_SOUNDS_ON_SD              "Keine T\203ne" BREAKSPACE "auf SD"
#define TR_NO_MODELS_ON_SD              "Kein Modelle" BREAKSPACE "auf SD"
#define TR_NO_BITMAPS_ON_SD             "Keine Bitmaps" BREAKSPACE "auf SD"
#define TR_NO_SCRIPTS_ON_SD             "Keine Skripte" BREAKSPACE "auf SD"
#define TR_SCRIPT_SYNTAX_ERROR          TR("Syntaxfehler", "Skript Syntaxfehler")
#define TR_SCRIPT_PANIC                 "Skript Panik"
#define TR_SCRIPT_KILLED                "Skript beendet"
#define TR_SCRIPT_ERROR                 "Unbekannter Fehler"
#define TR_PLAY_FILE                    "Abspielen"
#define TR_DELETE_FILE                  "L\203schen"
#define TR_COPY_FILE                    "Kopieren"
#define TR_RENAME_FILE                  "Umbenennen"
#define TR_ASSIGN_BITMAP                "Bitmap zuordnen"
#define TR_ASSIGN_SPLASH                "Als Startbild"
#define TR_EXECUTE_FILE                 "Execute"
#define TR_REMOVED                      " gel\203scht"
#define TR_SD_INFO                      "Information"
#define TR_SD_FORMAT                    "Formatieren"
#define TR_NA                           "N/V"	//NV=Nicht Verf\205gbar  Kurz-Meldung
#define TR_HARDWARE                     TR("Hardware einst. ", "Namen und Hardware einst.")
#define TR_FORMATTING                   "Formatierung..."
#define TR_TEMP_CALIB                   "Temp.  abgl."
#define TR_TIME                         "Uhrzeit:"
#define TR_MAXBAUDRATE                  "Max Baud"

#define TR_BLUETOOTH                    "Bluetooth"
#define TR_BLUETOOTH_DISC               "Discover"
#define TR_BLUETOOTH_INIT               "Init"
#define TR_BLUETOOTH_DIST_ADDR          "Dist addr"
#define TR_BLUETOOTH_LOCAL_ADDR         "Local addr"
#define TR_BLUETOOTH_PIN_CODE           "PIN code"
#define TR_BAUDRATE                     "BT Baudrate"
#define LEN_BLUETOOTH_MODES             "\011"
#if defined(PCBX9E)
#define TR_BLUETOOTH_MODES              "---\0     ""Enabled\0 "
#else
#define TR_BLUETOOTH_MODES              "---\0     ""Telemetry""Trainer\0"
#endif
#define TR_SD_INFO_TITLE               "SD-INFO"
#define TR_SD_TYPE                     "Typ:"
#define TR_SD_SPEED                    "Geschw:"
#define TR_SD_SECTORS                  "Sectoren:"
#define TR_SD_SIZE                     "Gr\203\206e:"
#define TR_TYPE                        INDENT "Typ"
#define TR_GLOBAL_VARS                 "Globale Variablen"
#define TR_GVARS                       "GLOBALE V."
#define TR_GLOBAL_VAR                  "Globale Variable"
#define TR_MENUGLOBALVARS              "GLOBALE VARIABLEN"
#define TR_OWN                         "Eigen"
#define TR_DATE                        "Datum:"
#define TR_MONTHS                      { "Jan", "Feb", "Mar", "Apr", "Mai", "Jun", "Jul", "Aug", "Sep", "Okt", "Nov", "Dez" }
#define TR_ROTARY_ENCODER              "Drehgeber"
#define TR_INVERT_ROTARY               "Invert Rotary"
#define TR_CHANNELS_MONITOR            "Kanal-Monitor==>"
#define TR_MIXERS_MONITOR              "==>Mischer Monitor"
#define TR_PATH_TOO_LONG               "Pfad zu Lang"
#define TR_VIEW_TEXT                   "View Text"
#define TR_FLASH_BOOTLOADER            TR("Flash bootloader","Flash bootloader")      //
#define TR_FLASH_EXTERNAL_DEVICE       TR("Flash ext. Ger\201t","Flash externes Ger\201t")
#define TR_FLASH_RECEIVER_OTA          "Flash receiver OTA"
#define TR_FLASH_RECEIVER_BY_EXTERNAL_MODULE_OTA "Flash RX by ext. OTA"
#define TR_FLASH_RECEIVER_BY_INTERNAL_MODULE_OTA "Flash RX by int. OTA"
#define TR_FLASH_FLIGHT_CONTROLLER_BY_EXTERNAL_MODULE_OTA "Flash FC by ext. OTA"
#define TR_FLASH_FLIGHT_CONTROLLER_BY_INTERNAL_MODULE_OTA "Flash FC by int. OTA"
#define TR_FLASH_BLUETOOTH_MODULE      TR("Flash BT module", "Flash Bluetoothmodul")
#define TR_FLASH_POWER_MANAGEMENT_UNIT TR("Flash pwr mngt unit", "Flash power management unit")
#define TR_CURRENT_VERSION             TR("Current vers. ", "Current version: ")
#define TR_HW_REV                      "HW Rev"
#define TR_FLASH_INTERNAL_MODULE       TR("Flash int. XJT","Flash internes XJT-Modul")
#define TR_FLASH_INTERNAL_MULTI        TR("Flash Int. Multi", "Flash int. Multimodul")
#define TR_FLASH_EXTERNAL_MODULE       TR("Flash ext. mod","Flash ext. Modul")
#define TR_FLASH_EXTERNAL_MULTI        TR("Flash Ext. Multi", "Flash ext. Multimodul")
#define TR_FLASH_EXTERNAL_ELRS         TR("Flash Ext. ELRS", "Flash External ELRS")
#define TR_FIRMWARE_UPDATE_ERROR       TR("FW update Error","Firmware Updatefehler")
#define TR_FIRMWARE_UPDATE_SUCCESS     TR3("Flash successful","Flash successful","Update erfolgreich")
#define TR_WRITING                     "Schreibe..."
#define TR_CONFIRM_FORMAT              "Formatieren best\201tigen?"
#define TR_INTERNALRF                  "Internes HF-Modul"
#define TR_INTERNAL_MODULE             TR("Int. module","Internal module")
#define TR_EXTERNAL_MODULE             TR("Ext. module","External module")
#define TR_OPENTX_UPGRADE_REQUIRED     "OpenTX upgrade n\203tig"
#define TR_TELEMETRY_DISABLED          "Telem. disabled"
#define TR_MORE_OPTIONS_AVAILABLE      "More options available"
#define TR_NO_MODULE_INFORMATION       "No module information"
#define TR_EXTERNALRF                  "Externes HF-Modul"
#define TR_FAILSAFE                    TR(INDENT "Failsafe", INDENT "Failsafe Mode")
#define TR_FAILSAFESET                 "Failsafe setzen"
#define TR_REG_ID                      TR("Reg. ID", "Registration ID")
#define TR_OWNER_ID                    "Owner ID"
#define TR_PINMAPSET                   "PINMAP"
#define TR_HOLD                        "Hold"
#define TR_HOLD_UPPERCASE              "HOLD"
#define TR_NONE                        "None"
#define TR_NONE_UPPERCASE              "NONE"
#define TR_MENUSENSOR                  "SENSOR"
#define TR_POWERMETER_PEAK             "Peak"
#define TR_POWERMETER_POWER            "Power"
#define TR_POWERMETER_ATTN             "Attn"
#define TR_POWERMETER_FREQ             "Freq."
#define TR_MENUTOOLS                   "TOOLS"
#define TR_TURN_OFF_RECEIVER           "Empf. ausschalten"
#define TR_STOPPING                    "Stopping..."
#define TR_MENU_SPECTRUM_ANALYSER      "SPECTRUM ANALYSER"
#define TR_MENU_POWER_METER            "POWER METER"
#define TR_SENSOR                      "SENSOR"
#define TR_COUNTRYCODE                 "Landescode"
#define TR_USBMODE                     "USB Modus"
#define TR_JACKMODE                    "Jack Mode"
#define TR_VOICELANG                   "Sprachansagen"
#define TR_UNITSSYSTEM                 "Einheiten"
#define TR_EDIT                        "Zeile Editieren"
#define TR_INSERT_BEFORE               "Neue Zeile davor"
#define TR_INSERT_AFTER                "Neue Zeile danach"
#define TR_COPY                        "Zeile kopieren"
#define TR_MOVE                        "Zeile verschieben"
#define TR_PASTE                       "Zeile einf\205gen"
#define TR_DELETE                      "Zeile l\203schen"
#define TR_INSERT                      "Neue Zeile"
#define TR_RESET_FLIGHT                "Reset Flugdaten"
#define TR_RESET_TIMER1                "Reset Timer1"
#define TR_RESET_TIMER2                "Reset Timer2"
#define TR_RESET_TIMER3	               "Reset Timer3"
#define TR_RESET_TELEMETRY             "Reset Telemetrie"
#define TR_STATISTICS                  "Statistik Timer Gas"
#define TR_ABOUT_US                    "Die Programmierer"
#define TR_USB_JOYSTICK                "USB Joystick (HID)"
#define TR_USB_MASS_STORAGE            "USB Speicher (SD)"
#define TR_USB_SERIAL                  "USB Seriell (Debug)"
#define TR_USB_TELEMETRY               "USB Telem mirror"
#define TR_SETUP_SCREENS               "Setup Hautbildschirme"
#define TR_MONITOR_SCREENS             "Monitore Mischer Kanal Logik"
#define TR_AND_SWITCH                  "UND Schalt" //UND mit weiterem Schaltern
#define TR_SF                          "SF" //Spezial Funktionen
#define TR_GF                          "GF" // Globale Funktionen
#define TR_SPEAKER                     INDENT "Lautspr"
#define TR_BUZZER                      INDENT "Summer"
#define TR_BYTES                       "Bytes"
#define TR_MODULE_BIND                 BUTTON(TR("Bnd","Bind"))   //9XR-Pro
#define TR_POWERMETER_ATTN_NEEDED      "Attenuator needed"
#define TR_PXX2_SELECT_RX              "Select RX"
#define TR_PXX2_DEFAULT                "<default>"
#define TR_BT_SELECT_DEVICE            "Select device"
#define TR_DISCOVER             		"Discover"
#define TR_BUTTON_INIT                 BUTTON("Init")
#define TR_WAITING                     "Waiting..."
#define TR_RECEIVER_DELETE             "Delete receiver?"
#define TR_RECEIVER_RESET              "Reset receiver?"
#define TR_SHARE                       "Share"
#define TR_BIND                        "Bind"
#define TR_REGISTER             	   TR("Reg", "Register")
#define TR_MODULE_RANGE        		   BUTTON(TR("Rng", "Range"))  //9XR-Pro
#define TR_RECEIVER_OPTIONS            TR("REC. OPTIONS", "RECEIVER OPTIONS")
#define TR_DEL_BUTTON                  BUTTON(TR("Del", "Delete"))
#define TR_RESET_BTN           		   BUTTON("Reset")
#define TR_DEBUG                       "Testen"
#define TR_KEYS_BTN                	   BUTTON(TR("SW","Switches"))
#define TR_ANALOGS_BTN                 BUTTON(TR("Analog","Analogs"))
#define TR_TOUCH_NOTFOUND              "Touch hardware not found"
#define TR_TOUCH_EXIT                  "Touch screen to exit"
#define TR_CALIBRATION                 "Kalibrieren"
#define TR_SET                   	   BUTTON("Set")
#define TR_TRAINER             		   "DSC Buchse PPM In/Out"
#define TR_CHANS                       "Chans"
#define TR_ANTENNAPROBLEM     		   CENTER "TX-Antennenproblem!"
#if defined(COLORLCD)
  #define TR_MODELIDUSED               "ID used in:"
#else
  #define TR_MODELIDUSED               TR("ID used in:","Receiver ID used in:")
#endif
#define TR_MODULE             		   "Modul-Typ"
#define TR_RX_NAME                     "Rx Name"
#define TR_TELEMETRY_TYPE      		   TR("Typ", "Telemetrietyp")
#define TR_TELEMETRY_SENSORS  		   "Sensoren"
#define TR_VALUE               		   "Wert"
#define TR_TOPLCDTIMER        		   "Top LCD Timer"
#define TR_UNIT                			"Einheit"
#define TR_TELEMETRY_NEWSENSOR          INDENT "Sensor hinzuf\205gen ..."
#define TR_CHANNELRANGE        			TR(INDENT "Kan\201le", INDENT "Ausgangs Kan\201le")  //wg 9XR-Pro
#define TR_AFHDS3_RX_FREQ              TR("RX freq.", "RX frequency")
#define TR_AFHDS3_ONE_TO_ONE_TELEMETRY TR("Unicast/Tel.", "Unicast/Telemetry")
#define TR_AFHDS3_ONE_TO_MANY          "Multicast"
#define TR_AFHDS3_ACTUAL_POWER         TR("Act. pow", "Actual power")
#define TR_AFHDS3_POWER_SOURCE         TR("Power src.", "Power source")
#define TR_ANTENNACONFIRM1     "Ant. umschalten"
#if defined(PCBX12S)
#define LEN_ANTENNA_MODES      "\023"
#define TR_ANTENNA_MODES       "Internal\0          ""Ask\0               ""Per model\0         ""Internal + External"
#else
#define LEN_ANTENNA_MODES      "\011"
#define TR_ANTENNA_MODES       "Internal\0""Ask\0     ""Per model""External"
#endif
#define TR_USE_INTERNAL_ANTENNA        TR("Use int. antenna", "Use internal antenna")
#define TR_USE_EXTERNAL_ANTENNA        TR("Use ext. antenna", "Use external antenna")
#define TR_ANTENNACONFIRM2     		   TR("Check antenna", "Ist eine externe Antenne installiert?")
#define TR_MODULE_PROTOCOL_FLEX_WARN_LINE1   "Requires non"
#define TR_MODULE_PROTOCOL_FCC_WARN_LINE1    "Requires FCC"
#define TR_MODULE_PROTOCOL_EU_WARN_LINE1     "Requires EU"
#define TR_MODULE_PROTOCOL_WARN_LINE2        "certified firmware"
#define TR_LOWALARM                    INDENT "1.Warnschwelle"
#define TR_CRITICALALARM               INDENT "Kritischer Alarm"
#define TR_RSSIALARM_WARN              "RSSI"
#define TR_NO_RSSIALARM                TR(INDENT "RSSI Alarms AUS", "RSSI Alarme ausschalten")
#define TR_DISABLE_ALARM               TR(INDENT "Alarme AUS", INDENT "Telemetrie Alarme AUS")
#define TR_ENABLE_POPUP                "Freigabe Popup-Fenster"
#define TR_DISABLE_POPUP               "Sperren  Popup-Fenster"
#define TR_POPUP                       "Popup"
#define TR_MIN                         "Min"
#define TR_MAX                         "Max"
#define TR_CURVE_PRESET                "Gerade 0 11 22 33 45"
#define TR_PRESET                      "Preset"
#define TR_MIRROR                      "Spiegeln"
#define TR_CLEAR                       "L\203schen"
#define TR_RESET                       TR("Servowert reset","Servowerte zur\205cksetzen")
#define TR_RESET_SUBMENU               TR("Reset Werte   ==>", "Reset=>Timer Flug Telem")
#define TR_COUNT                       "Punkte"
#define TR_PT                          "Pt"
#define TR_PTS                         "Pts"
#define TR_SMOOTH                      "Runden"
#define TR_COPY_STICKS_TO_OFS          TR3("Copy Stk ->Subtrim", "Kopiere Stick zu Subtrim", "Kopiere Kn\205ppelposition auf Subtrim")
#define TR_COPY_MIN_MAX_TO_OUTPUTS     TR3("Cpy min/max to all", "Kopiere min/max zu allen" , "Kopiere Limits & Mitte auf alle Kan\201le")
#define TR_COPY_TRIMS_TO_OFS           TR3("Copy Trim->Subtrim",  "Kopiere Trimm zu Subtrim" , "Kopiere Trimmposition auf Subtrim")  // "Trim to Subtrim"
#define TR_INCDEC                      "Inc/Decrement"
#define TR_GLOBALVAR                   "Global Var"
#define TR_MIXSOURCE                   "Mixer Quelle"
#define TR_CONSTANT                    "Konstant"
#define TR_PERSISTENT_MAH              TR(INDENT "Spr. mAh", INDENT "Speichern mAh") //9XR-Pro
#define TR_PREFLIGHT                   "Vorflug-Checkliste"
#define TR_CHECKLIST                   TR(INDENT "Checkliste", INDENT "Checkliste anzeigen") //9XR-Pro
#define TR_FAS_OFFSET                  TR(INDENT "FAS-Ofs", INDENT "FAS-Offset")
#define TR_AUX_SERIAL_MODE             "Serieller Port"
#define TR_AUX2_SERIAL_MODE            "Serieller Port 2"
#define TR_SCRIPT                      "Lua-Skript"
#define TR_INPUTS                      "Eingaben"
#define TR_OUTPUTS                     "Ausgaben"
#define STR_EEBACKUP                    TR("EEPROM->SD", "Backup EEPROM->SD-Karte")
#define STR_FACTORYRESET                TR("Werksreset", "Auf Werkseinstellungen")
#define TR_CONFIRMRESET                 TR("Alles l\203schen? ","ALLE Modelle+Einst. l\203schen?")
#define TR_TOO_MANY_LUA_SCRIPTS         "Zu viele Skripte!"
#define TR_SPORT_UPDATE_POWER_MODE      "SP Power"
#define LEN_SPORT_UPDATE_POWER_MODES    "\004"
#define TR_SPORT_UPDATE_POWER_MODES     "AUTO""ON\0 "
#define TR_NO_TELEMETRY_SCREENS         "No Telemetry Screens"
#define TR_TOUCH_PANEL                 "Touch panel:"

// Horus and Taranis specific column headers
#define TR_PHASES_HEADERS_NAME         "Name "
#define TR_PHASES_HEADERS_SW           "Schalter"
#define TR_PHASES_HEADERS_RUD_TRIM     "Trim Seite"
#define TR_PHASES_HEADERS_ELE_TRIM     "Trim H\203he"
#define TR_PHASES_HEADERS_THT_TRIM     "Trim Gas"
#define TR_PHASES_HEADERS_AIL_TRIM     "Trim Quer"
#define TR_PHASES_HEADERS_CH5_TRIM     "Trim 5"
#define TR_PHASES_HEADERS_CH6_TRIM     "Trim 6"
#define TR_PHASES_HEADERS_FAD_IN       "Langs Ein"
#define TR_PHASES_HEADERS_FAD_OUT      "Langs Aus"

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

// Taranis Info Zeile Anzeigen
#define TR_LIMITS_HEADERS_NAME         "Name"
#define TR_LIMITS_HEADERS_SUBTRIM      "Subtrim"
#define TR_LIMITS_HEADERS_MIN          "Min"
#define TR_LIMITS_HEADERS_MAX          "Max"
#define TR_LIMITS_HEADERS_DIRECTION    "Richtung"
#define TR_LIMITS_HEADERS_CURVE        "Kurve"
#define TR_LIMITS_HEADERS_PPMCENTER    "PPM Mitte"
#define TR_LIMITS_HEADERS_SUBTRIMMODE  "Subtrim Mode"

#define TR_LSW_HEADERS_FUNCTION        "Funktion"
#define TR_LSW_HEADERS_V1              "V1"
#define TR_LSW_HEADERS_V2              "V2"
#define TR_LSW_HEADERS_ANDSW           "UND Schalter"
#define TR_LSW_HEADERS_DURATION        "Dauer"
#define TR_LSW_HEADERS_DELAY           "Verz\203gerung"

#define TR_GVAR_HEADERS_NAME           "Name"
#define TR_GVAR_HEADERS_FM0            "Wert im FM0"
#define TR_GVAR_HEADERS_FM1            "Wert im FM1"
#define TR_GVAR_HEADERS_FM2            "Wert im FM2"
#define TR_GVAR_HEADERS_FM3            "Wert im FM3"
#define TR_GVAR_HEADERS_FM4            "Wert im FM4"
#define TR_GVAR_HEADERS_FM5            "Wert im FM5"
#define TR_GVAR_HEADERS_FM6            "Wert im FM6"
#define TR_GVAR_HEADERS_FM7            "Wert im FM7"
#define TR_GVAR_HEADERS_FM8            "Wert im FM8"

// Horus footer descriptions
#define TR_LSW_DESCRIPTIONS            { "Vergleich oder Funktion", "Erste Variable", "Zweite Variable/Konstante", "Zweite Variable/Konstante", "Weitere UND Bedingung f\205r Freigabe des Log Schalters", "ON-Zeit des Log Schalters wenn Bedingung ok", "Mindestdauer der Bedingung damit Log Schalter ON geht" }

//Taranis About screen
#define TR_ABOUTUS                     "\204ber OpenTx"

#define TR_ABOUT_OPENTX_1              "OpenTX ist Open Source,"
#define TR_ABOUT_OPENTX_2              "nicht kommerziell, ohne"
#define TR_ABOUT_OPENTX_3              "Funktionsgarantie, frei"
#define TR_ABOUT_OPENTX_4              "verf\205gbar. Unterst\205tzung"
#define TR_ABOUT_OPENTX_5              "durch Spenden willkommen"

#define TR_ABOUT_BERTRAND_1            "Bertrand Songis"
#define TR_ABOUT_BERTRAND_2            "OpenTX Hauptauthor"
#define TR_ABOUT_BERTRAND_3            "Companion-Mitentwickler"

#define TR_ABOUT_MIKE_1                "Mike Blandford"
#define TR_ABOUT_MIKE_2                "Code- und Treiber-Guru"
#define TR_ABOUT_MIKE_3                "Wohl einer der Besten."
#define TR_ABOUT_MIKE_4                "Sehr inspirierend."

#define TR_ABOUT_ROMOLO_1              "Romolo Manfredini"
#define TR_ABOUT_ROMOLO_2              "Companion-Hauptentwickler"
#define TR_ABOUT_ROMOLO_3              ""

#define TR_ABOUT_ANDRE_1               "Andre Bernet"
#define TR_ABOUT_ANDRE_2               "Funktionalit\201t und Tests,"
#define TR_ABOUT_ANDRE_3               "Debugging, Dokumentation"

#define TR_ABOUT_ROB_1                 "Rob Thomson"
#define TR_ABOUT_ROB_2                 "Openrcforums Webmaster"

#define TR_ABOUT_KJELL_1               "Kjell Kernen"
#define TR_ABOUT_KJELL_2               "www.open-tx.org Hauptautor"
#define TR_ABOUT_KJELL_3               "Author von OpenTX Recorder"
#define TR_ABOUT_KJELL_4               "Companion contributor"

#define TR_ABOUT_MARTIN_1              "Martin Hotar"
#define TR_ABOUT_MARTIN_2              "Grafikdesigner"

#if defined(MANUFACTURER_FRSKY)
  #define TR_ABOUT_HARDWARE_1          "FrSky"
  #define TR_ABOUT_HARDWARE_2          TR("Hardware design/producer", "Hardware designer/producer")
  #define TR_ABOUT_HARDWARE_3          "Firmware contributor"
#elif defined(MANUFACTURER_RADIOMASTER)
  #define TR_ABOUT_HARDWARE_1          "Radiomaster"
  #define TR_ABOUT_HARDWARE_2          TR("Hardware design/producer", "Hardware designer/producer")
  #define TR_ABOUT_HARDWARE_3          "Firmware contributor"
#elif defined(MANUFACTURER_JUMPER)
  #define TR_ABOUT_HARDWARE_1          "JumperRC"
  #define TR_ABOUT_HARDWARE_2          TR("Hardware design/producer", "Hardware designer/producer")
  #define TR_ABOUT_HARDWARE_3          "Firmware contributor"
#else
  #define TR_ABOUT_HARDWARE_1         "Brent Nelson"
  #define TR_ABOUT_HARDWARE_2         "Sky9x designer/producer"
  #define TR_ABOUT_HARDWARE_3         ""
#endif

#define TR_ABOUT_PARENTS_1            "Vorg\201nger-Projekte"
#define TR_ABOUT_PARENTS_2            "Ersky9x (Mike Blandford)"
#define TR_ABOUT_PARENTS_3            "ER9X (Erez Raviv)"
#define TR_ABOUT_PARENTS_4            "TH9X (Thomas Husterer)"

#define TR_CHR_SHORT  				  's' // Taste short
#define TR_CHR_LONG   			      'l' // Taste long
#define TR_CHR_TOGGLE 				  't' // Taste als togglefunktion = Ein Aus Ein
#define TR_CHR_HOUR   				  'h' // Stunden
#define TR_CHR_INPUT  				  'I' // Values between A-I will work

#define TR_BEEP_VOLUME                 "Beep-Lautst."
#define TR_WAV_VOLUME                  "Wav-Lautst."
#define TR_BG_VOLUME                   TR("Bgr-Lautst.", "Hintergrund-Lautst\201rke")

#define TR_TOP_BAR                     "Infozeile"
#define TR_FLASH_ERASE                 "Flash erase..."
#define TR_FLASH_WRITE                 "Flash write..."
#define TR_OTA_UPDATE                  "OTA update..."
#define TR_MODULE_RESET                "Module reset..."
#define TR_UNKNOWN_RX                  "Unknown RX"
#define TR_UNSUPPORTED_RX              "Unsupported RX"
#define TR_OTA_UPDATE_ERROR            "OTA update error"
#define TR_DEVICE_RESET                "Device reset..."
#define TR_ALTITUDE                    INDENT "H\203henanzeige"
#define TR_SCALE                       "Skalieren"
#define TR_VIEW_CHANNELS               "Zeige Kan\201le"
#define TR_VIEW_NOTES                  "Zeige Notizen"
#define TR_MODEL_SELECT                "Modell ausw\201hlen"
#define TR_MODS_FORBIDDEN              "Anpassung verboten!"
#define TR_UNLOCKED                    "Entsperrt"
#define TR_ID                          "ID"
#define TR_PRECISION                   "Pr\201zision"
#define TR_RATIO                       "Umrechnung"  //Faktor, Mulitplikator, Teiler  0,1 bis 10,0
#define TR_FORMULA                     "Formel"
#define TR_CELLINDEX                   "Zellenindex"
#define TR_LOGS                        "Log Daten"
#define TR_OPTIONS                     "Optionen"
#define TR_FIRMWARE_OPTIONS            "Firmwareoptionen"

#define TR_ALTSENSOR                   "H\203hen Sensor"
#define TR_CELLSENSOR                  "Zellen Sensor"
#define TR_GPSSENSOR                   "GPS Sensor"
#define TR_CURRENTSENSOR               "Sensor"
#define TR_AUTOOFFSET                  "Auto Offset"
#define TR_ONLYPOSITIVE                "Nur Positiv"
#define TR_FILTER                      "Filter aktiv"
#define TR_TELEMETRYFULL               TR("Telem voll!", "Telemetriezeilen voll!")
#define TR_SERVOS_OK                   "Servos OK"
#define TR_SERVOS_KO                   "Servos KO"
#define TR_INVERTED_SERIAL             INDENT "Invert."
#define TR_IGNORE_INSTANCE             TR(INDENT "No Inst.", INDENT "Ignor. Instanzen")
#define TR_DISCOVER_SENSORS            "Start Sensorsuche"
#define TR_STOP_DISCOVER_SENSORS       "Stop Sensorsuche"
#define TR_DELETE_ALL_SENSORS          "L\203sche alle Sensoren"
#define TR_CONFIRMDELETE               "Wirklich alle " LCDW_128_480_LINEBREAK "l\203schen ?"
#define TR_SELECT_WIDGET               "Widget ausw\201hlen"  // grafisches Element
#define TR_REMOVE_WIDGET               "Widget l\203schen"
#define TR_WIDGET_SETTINGS             "Widget einstellen"
#define TR_REMOVE_SCREEN               "Screen l\203schen"
#define TR_SETUP_WIDGETS               "Setup widgets"
#define TR_USER_INTERFACE              "User interface"
#define TR_THEME                       "Theme"
#define TR_SETUP                       "Setup"
#define TR_MAINVIEWX                   "Ansicht X"
#define TR_LAYOUT                      "Layout"
#define TR_ADDMAINVIEW                 "Ansicht hinzuf\205gen"
#define TR_BACKGROUND_COLOR            "Hintergrundfarbe"
#define TR_MAIN_COLOR                  "Hauptfarbe"
#define TR_BAR2_COLOR                  "Secondary bar color"
#define TR_BAR1_COLOR                  "Main bar color"
#define TR_TEXT_COLOR                  "Text color"
#define TR_TEXT_VIEWER                 "Text Viewer"
// ----------------------------- Symbole f\205r Auswahlliste----------
#define TR_MENU_INPUTS                 "\314Inputs"
#define TR_MENU_LUA                    "\322Lua Skripte"
#define TR_MENU_STICKS                 "\307Kn\205ppel"
#define TR_MENU_POTS                   "\310Potis"
#define TR_MENU_MAX                    "\315MAX"
#define TR_MENU_HELI                   "\316Heli-TS CYC1-3"
#define TR_MENU_TRIMS                  "\313Trimmung"
#define TR_MENU_SWITCHES               "\312Schalter"
#define TR_MENU_LOGICAL_SWITCHES       "\312Log. Schalter"
#define TR_MENU_TRAINER                "\317Trainer"
#define TR_MENU_CHANNELS               "\320Kan\201le"
#define TR_MENU_GVARS                  "\311Glob. Vars"
#define TR_MENU_TELEMETRY              "\321Telemetrie"
#define TR_MENU_DISPLAY                "TELM-SEITEN"
#define TR_MENU_OTHER                  "Weitere"
#define TR_MENU_INVERT                 "Invertieren<!>"
#define TR_JITTER_FILTER               "ADC Filter"
#define TR_RTC_CHECK                   TR("Check RTC", "Check RTC voltage")
#define TR_AUTH_FAILURE                "Auth-failure"
#define TR_RACING_MODE                 "Racing mode"

// ----------------------------------------------------------------
#define ZSTR_VFR                       "\026\006\022"
#define ZSTR_RSSI                      "\022\023\023\011"
#define ZSTR_R9PW                      "\022\044\020\027"
#define ZSTR_RAS                       "\023\027\022"
#define ZSTR_A1                        "\001\034"
#define ZSTR_A2                        "\001\035"
#define ZSTR_A3                        "\001\036"
#define ZSTR_A4                        "\001\037"
#define ZSTR_BATT                      "\022\350\002\354"
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
#define ZSTR_FLIGHT_MODE               "\006\015"
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
#define ZSTR_HOTT_ID_EAM_SPEED      "\005\001\355p"
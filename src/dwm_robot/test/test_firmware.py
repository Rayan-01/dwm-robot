"""Host-side C++ tests of actual firmware command parser/watchdog.

This verifies protocol logic, not AVR compilation or physical pin operation.
"""

import shutil
import re
import subprocess
from pathlib import Path
import tempfile
import unittest


class FirmwareTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which("g++"), "g++ not available")
    def test_stop_latch_relay_timeout_and_strict_parser(self):
        root = Path(__file__).resolve().parents[3]
        source = (root / "firmware/dwm_robot_safe/dwm_robot_safe.ino").read_text()
        block = source[source.index("void allStop()") : source.index("// READ SERIAL")]
        watch = source[
            source.index("  if(permissionEnabled && millis()") : source.index(
                "  updateMotors();", source.index("void loop()")
            )
        ]
        stub = r"""
#include <string>
#include <algorithm>
#include <cassert>
#include <iostream>
struct String {
 std::string s;
 String(const char* x=""):s(x){} String(std::string x):s(x){}
 void trim(){auto a=s.find_first_not_of(" \r\n\t");auto b=s.find_last_not_of(" \r\n\t");s=a==std::string::npos?"":s.substr(a,b-a+1);}
 unsigned int length()const{return s.size();} char operator[](int i)const{return s[i];}
 bool operator==(const char* x)const{return s==x;}
 bool startsWith(const char* x)const{return s.rfind(x,0)==0;}
 int indexOf(char c,int at)const{auto p=s.find(c,at);return p==std::string::npos?-1:p;}
 String substring(int a,int b)const{return s.substr(a,b-a);}
 String substring(int a)const{return s.substr(a);}
 long toInt()const{try{return std::stol(s);}catch(...){return 0;}}
};
struct SerialStub {std::string last; void println(const char* x){last=x;}} Serial;
bool permissionEnabled=false; int targetM1=0,targetM2=0,relayMode=0;
const int RELAY_PIN=11;
unsigned long clockMS=0,lastCommand=0; const unsigned long COMMAND_TIMEOUT=500;
int relayPin=0,coasts=0;
unsigned long millis(){return clockMS;}
void digitalWrite(int pin,int value){if(pin==RELAY_PIN)relayPin=value;}
void coastBoth(){coasts++;}
void printTelemetry(){}
void setTargets(int l,int r){targetM1=l;targetM2=r;lastCommand=millis();}
"""
        main = r"""
int main(){
 processCommand("M,50,50"); assert(targetM1==0&&!permissionEnabled);
 processCommand("E"); assert(permissionEnabled);
 processCommand("M,50,-45"); assert(targetM1==50&&targetM2==-45);
 processCommand("R,1"); assert(relayPin==RELAY_ON_LEVEL);
 processCommand("S"); assert(!permissionEnabled&&targetM1==0&&targetM2==0&&relayPin==RELAY_OFF_LEVEL);
 processCommand("M,90,90"); assert(targetM1==0);
 processCommand("E"); processCommand("R,A"); assert(!permissionEnabled&&relayPin==RELAY_OFF_LEVEL);
 processCommand("E"); processCommand("M,55oops,55"); assert(!permissionEnabled&&targetM1==0);
 processCommand("E"); processCommand("M,256,20"); assert(!permissionEnabled&&targetM1==0);
 processCommand("E"); processCommand("M,45,45");processCommand("R,1");
 clockMS=501; watchdog();assert(!permissionEnabled&&targetM1==0&&targetM2==0&&relayPin==RELAY_OFF_LEVEL);
 processCommand("E");processCommand("M,40,40");processCommand("E"); assert(!permissionEnabled&&targetM1==0);
 processCommand("V");assert(Serial.last=="K,DWM2");
 std::cout << "Actual firmware parser/watchdog assertions passed\n";
}
"""
        with tempfile.TemporaryDirectory() as directory:
            cpp = Path(directory) / "protocol.cpp"
            exe = Path(directory) / "protocol"
            levels = []
            for name in ("RELAY_ON_LEVEL", "RELAY_OFF_LEVEL"):
                level = re.search(
                    r"const bool " + name + r"\s*=\s*(HIGH|LOW);", source
                ).group(1)
                levels.append(
                    "const int "
                    + name
                    + "="
                    + ("1" if level == "HIGH" else "0")
                    + ";\n"
                )
            cpp.write_text(
                "".join(levels)
                + stub
                + block
                + "\nvoid watchdog(){\n"
                + watch
                + "\n}\n"
                + main
            )
            subprocess.run(
                ["g++", "-std=c++11", "-Wall", "-Wextra", str(cpp), "-o", str(exe)],
                check=True,
                capture_output=True,
            )
            result = subprocess.run(
                [str(exe)], check=True, capture_output=True, text=True
            )
            self.assertIn("passed", result.stdout)


if __name__ == "__main__":
    unittest.main()

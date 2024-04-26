# -*- coding: utf-8 -*-
"""
This module is used to upload the server (and logger) script(s) to the ABB
controller, and start the server running on the controller. It is based on
the ABBcontroller.py and ABBcalibration.py scripts that performed similar
functions in the legacy code.

"""

from System.IO import Directory
import sys
import time
import clr
import warnings

warnings.simplefilter("ignore", DeprecationWarning)

clr.AddReferenceToFileAndPath("dlls\ABB.Robotics.Controllers.PC.dll")
import ABB.Robotics.Controllers as Controllers

def scan():
    scanner = Controllers.Discovery.NetworkScanner()
    scanner.Scan()
    print "\n" + str(scanner.Controllers.Count) + " ABB robots found on network:\n"
    if scanner.Controllers.Count == 0:
        while(True):
            input_char = raw_input("Press R to rescan, Q to quit\n")
            if input_char.upper() == 'Q':
                sys.exit()
            elif input_char.upper() == 'R':
                return scan()
            else:
                print "Wrong input"
    else:
        while True:
            # print "Select ABB robot number:"
            for i in range(len(scanner.Controllers)):
                virtual = "Real"
                if scanner.Controllers[i].IsVirtual:
                    virtual = "Virtual"
                print "[" + str(i+1) + "] " + scanner.Controllers[i].IPAddress.ToString() + " " + virtual + "\n"
            robotNumber = int(raw_input("Select ABB robot number: ")) - 1
            print
            try:
                controller = scanner.Controllers[robotNumber]
                break
            except Exception as e:
                print e

        return controller

if __name__ == "__main__":      
    
    def end_session():
         print "Disconnecting from ABB robot arm\n"
         MyController.Rapid.Stop(Controllers.RapidDomain.StopMode.Immediate)      
         Mastership.Release()
         Mastership.Dispose()
         MyController.Logoff()
         MyController.Dispose()        
    
    def close():
        end_session()
        exit()

    controller = scan()
 
    MyController = Controllers.ControllerFactory.CreateFrom(controller)
    MyController.Logon(Controllers.UserInfo.DefaultUser)
    try:
        Mastership = Controllers.Mastership.Request(MyController.Rapid)
        print "Mastership of virtual controller granted ...\n"
    except Exception as e:
        print e
    MyController.AuthenticationSystem.CheckDemandGrant(Controllers.Grant.ExecuteRapid)
    print "Authentication granted to run RAPID Code ...\n"

    try:
        Mastership
        
        MyController.FileSystem.RemoteDirectory = MyController.GetEnvironmentVariable("HOME")
        MyController.FileSystem.LocalDirectory = Directory.GetCurrentDirectory()
        
        filename = "SERVER.mod"        
        MyController.FileSystem.PutFile(filename, filename, True)
        tRob1 = MyController.Rapid.GetTask("T_ROB1")
        tRob1.DeleteProgram()
        tRob1.LoadModuleFromFile(MyController.GetEnvironmentVariable("HOME") + "/" + filename, Controllers.RapidDomain.RapidLoadMode.Replace)
        tRob1.SetProgramPointer("SERVER", "main")
        ipController = MyController.IPAddress.ToString()
        MyController.Rapid.GetRapidData("T_ROB1", "SERVER", "ipController").Value = Controllers.RapidDomain.String(ipController)        
        
        with open("controller.ini", "w") as ifile:
            ifile.write("ip_address={0}".format(ipController))
        
#        filename = "LOGGER.mod"
#        MyController.FileSystem.PutFile(filename, filename, True)        
#        tRob2 = MyController.Rapid.GetTask("T_ROB2")
#        tRob2.DeleteProgram()
#        tRob2.LoadModuleFromFile(MyController.GetEnvironmentVariable("HOME") + "/" + filename, Controllers.RapidDomain.RapidLoadMode.Replace)
#        tRob2.SetProgramPointer("LOGGER", "main")

        MyController.Rapid.Start()
#        print "Server and logger started (use CTRL-C to terminate) ...\n"
        print "Server started (use CTRL-C to terminate) ...\n"        
        
    except Exception as e:
        print(e)

    try:
        while True:
            time.sleep(0.1)
    finally:
        end_session()

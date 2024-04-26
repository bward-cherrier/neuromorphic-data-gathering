import subprocess
import sys
import pprint

username = 'inivation'
extraPath = fr';C:\WINDOWS\System32\OpenSSH;C:\Program Files\WindowsPowerShell\Scripts;C:\Users\{username}\AppData\Local\Microsoft\WindowsApps'

cleanEnvironment = {
    'ALLUSERSPROFILE': 'C:\ProgramData',
    'APPDATA': fr'C:\Users\{username}\AppData\Roaming',
    'CommonProgramFiles': 'C:\Program Files\Common Files',
    'CommonProgramFiles(x86)': 'C:\Program Files (x86)\Common Files',
    'CommonProgramW6432': 'C:\Program Files\Common Files',
    'COMSPEC': 'C:\Windows\System32\cmd.exe',
    'DriverData': 'C:\Windows\System32\Drivers\DriverData',
    'HOMEDRIVE': 'C:',
    'HOMEPATH': fr'\Users\{username}',
    'LOCALAPPDATA': fr'C:\Users\{username}\AppData\Local',
    'OS': 'Windows_NT',
    'PATH':
    fr'C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\WINDOWS\System32\WindowsPowerShell\v1.0{extraPath}',
    'PATHEXT': '.COM;.EXE;.BAT;.CMD;.VBS;.JS;.WS;.MSC',
    'ProgramData': 'C:\ProgramData',
    'ProgramFiles': 'C:\Program Files',
    'ProgramFiles(x86)': 'C:\Program Files (x86)',
    'ProgramW6432': 'C:\Program Files',
    'PROMPT': '$P$G',
    'SystemDrive': 'C:',
    'SYSTEMROOT': 'C:\WINDOWS',
    'TEMP': fr'C:\Users\{username}\AppData\Local\Temp',
    'TMP': fr'C:\Users\{username}\AppData\Local\Temp',
    'USER': f'{username}',
    'USERNAME': f'{username}',
    'USERPROFILE': fr'C:\Users\{username}',
    'WINDIR': 'C:\WINDOWS',
    # System related variables.
    'NUMBER_OF_PROCESSORS': '8',
    'PROCESSOR_ARCHITECTURE': 'AMD64',
    'PROCESSOR_IDENTIFIER': 'Intel64 Family 6 Model 42 Stepping 7, GenuineIntel',
    'PROCESSOR_LEVEL': '6',
    'PROCESSOR_REVISION': '2a07'
}

pp = pprint.PrettyPrinter(indent=4)
print("Arguments: ", flush=True)
pp.pprint(sys.argv)
print("Username: " + username, flush=True)
print("Clean Environment: ", flush=True)
pp.pprint(cleanEnvironment)

p = subprocess.run(['C:\Windows\system32\cmd.exe', '/c', sys.argv[1]], capture_output=True, env=cleanEnvironment)

print("=== OUTPUTS ===", flush=True)
print('STDOUT: ' + p.stdout.decode('ansi', 'replace').replace('\r', ''), flush=True)
print('STDERR: ' + p.stderr.decode('ansi', 'replace').replace('\r', ''), flush=True)

if p.returncode != 0:
    raise RuntimeError(f'Failed with return code {p.returncode}')

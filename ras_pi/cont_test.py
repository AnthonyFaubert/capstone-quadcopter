import time
import subprocess

sc_proc = subprocess.Popen('sc-controller', stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
time.sleep(5)
sc_proc.kill()
js_proc = subprocess.Popen('jstest-gtk', stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
time.sleep(3)
js_proc.kill()

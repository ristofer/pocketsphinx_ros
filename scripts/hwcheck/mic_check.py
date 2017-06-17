from uchile_util.syscheck import SystemCheck, SystemCheckTask, FileCheckTask
from uchile_util import sh


class MicCheckTask(SystemCheckTask):
    def __init__(self, _name):
        super(MicCheckTask, self).__init__()
        self.name = _name


    def check(self):
        SystemCheck.print_high("Target alsa name : " + self.name, 1)

        cmd = "[ $(pacmd list-sources | grep -e '{}' | wc -l) = 1 ]".format(self.name)
        SystemCheck.print_info("Checking alsa devices... ", 1)
        if not sh.exec_cmd(cmd, level=1):
            SystemCheck.print_error("No connected device matches alsa name.", level=1)
            return False

        SystemCheck.print_ok("Device is connected.", 1)
        return True


def check():
    mic = SystemCheck("Microphone")
    mic.add_child(MicCheckTask("alsa_input.usb-M-Audio_Producer_USB-00-USB.analog-stereo"))
    return mic.check()

# Jonas Braun
# jonas.braun@tum.de
# 18.01.2019


# class to translate commands received via UDP to commands understandable by the BCI and vice versa
class BCICommands:
    def __init__(self):
        self.confirm = bytes(str(0), 'UTF-8')

    @staticmethod
    def screen(cmd):
        screen = int(cmd)
        return screen

    @staticmethod
    def result(result):
        cmd = result
        return cmd

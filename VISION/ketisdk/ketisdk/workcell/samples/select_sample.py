from ketisdk.utils.proc_utils import ProcUtils

def run():
    commands = ['workcell_demo_1']

    cmd_dict = ProcUtils().get_command_keys(commands)
    key = input('select action: \n')
    input_command = cmd_dict[key]

    if input_command == 'workcell_demo_1':
        print('this is work cell demo 1')

if __name__=='__main__':
    run()




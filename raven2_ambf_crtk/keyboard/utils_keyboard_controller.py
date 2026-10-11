import sys

RED = '\033[91m'
GREEN = '\033[92m'
RESET = '\033[0m'

def print_menu():
    menu_items = [
        ('Exit', '9'),
        ('Joint 1 +', '1', 'Joint 1 -', 'q'),
        ('Joint 2 +', '2', 'Joint 2 -', 'w'),
        ('Joint 3 +', '3', 'Joint 3 -', 'e'),
        ('Joint 4 +', '4', 'Joint 4 -', 'r'),
        ('Joint 5 +', '5', 'Joint 5 -', 't'),
        ('Joint 6 +', '6', 'Joint 6 -', 'y'),
        ('Joint 7 +', '7', 'Joint 7 -', 'u'),
        ('Grasper open', 'o'),
        ('Grasper close', 'p'),
        ('Grasper forward', 's'),
        ('Grasper backward', 'x'),
        ('Grasper left', 'z'),
        ('Grasper right', 'c'),
        ('Grasper up', 'f'),
        ('Grasper down', 'v'),
        ('Grasper P -', 'h'),
        ('Grasper P +', 'n'),
        ('Grasper R -', 'b'),
        ('Grasper R +', 'm'),
        ('Grasper Y -', 'g'),
        ('Grasper Y +', 'j'),
    ]

    print(f'\n{GREEN}-----------------------------------------{RESET}')
    print(f'{GREEN}AMBF RAVEN 2 Keyboard Controller:{RESET}')
    print(f'{GREEN}-----------------------------------------{RESET}')

    for item in menu_items:
        if len(item) == 2:
            print(f'{item[0]:<20} [{RED}{item[1]}{RESET}]')
        elif len(item) == 4:
            print(f'{item[0]:<20} [{RED}{item[1]}{RESET}] | {item[2]:<20} [{RED}{item[3]}{RESET}]')

    print(f'{GREEN}-----------------------------------------{RESET}')
    print(f'{GREEN}Tips: Press keys to control joints, press 9 to exit.{RESET}')
    print(f'{GREEN}-----------------------------------------{RESET}')
    print('Current command:')
    return None

def print_no_newline(string):
    sys.stdout.write(f"\r{GREEN}" + string + f"{RESET}")
    sys.stdout.flush()
    return None
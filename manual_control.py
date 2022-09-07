


import argparse


def main():
    argparser = argparse.ArgumentParser(description = 'CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug infomation')
    argparser.add_argument(
        '--host', metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default:127.0.0.1)')

    # https://blog.csdn.net/fireflylane/article/details/84575211?spm=1001.2101.3001.6650.1&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-84575211-blog-116785909.pc_relevant_multi_platform_featuressortv2dupreplace&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-84575211-blog-116785909.pc_relevant_multi_platform_featuressortv2dupreplace&utm_relevant_index=2
    print(__doc__)
    try:
        game_loop(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

if __name__ == '__main__':
    main()
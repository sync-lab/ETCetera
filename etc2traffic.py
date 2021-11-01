import re
import sys
import argparse
import os

from ETCetera.exceptions.sentient_base_exception import SentientBaseException




if __name__ == "__main__":
    # main(sys.argv)
    parser = argparse.ArgumentParser(prog='etc2pta', usage='python %(prog)s system_type input_file [options]')

    parser.add_argument('system_type', help='System type. Possible values: linear, nonlinear')
    parser.add_argument('input_file', help='Path to input file')
    parser.add_argument('-o', '--output_file', help='Output file, default is out.json if nothing specified. Tries to extract file type from the name if possible.', nargs='?', const='out.json')
    parser.add_argument('--output_type', nargs=1, help='Output file type. Will overwrite type generated from -o option')
    parser.add_argument('-v', '--verbose', help='Increase verbosity', action='count', default=0)

    args = parser.parse_args()
    # print(args)
    system_type = args.system_type.lower()
    input_file = args.input_file

    if system_type not in {'linear', 'non-linear', 'nonlinear'}:
        print(f'{system_type} is not a valid system type.')
        parser.print_help()
        sys.exit(os.EX_IOERR)

    if not os.path.exists(input_file):
        print(f'{input_file} is not a valid file.')
        parser.print_help()
        sys.exit(os.EX_IOERR)

    # if args.verbose:
    import logging
    lv = max(0, 40-10*args.verbose)
    logger = logging.getLogger()
    logger.setLevel(lv)
    ch = logging.StreamHandler()
    ch.setLevel(lv)
    ch.setFormatter(
        logging.Formatter('[%(levelname)s - %(filename)s:%(lineno)s - '
                          '%(funcName)s()] %(message)s'))
    if len(logger.handlers) <= 0:
        logger.addHandler(ch)

    # Process output file before the costly computation
    if args.output_file:
        re_string = '\.(json|pickle)$'
        temp = re.findall(re_string, args.output_file)
        if len(temp) > 1 or len(temp) == 0 and not args.output_type:
            print('Please specify a correct output file type.')

        out_type = temp[0]

        if args.output_type:
            # Replace the extension type
            args.output_file = args.output_file[0:-len(temp[0])] + args.output_type[0]
            out_type = args.output_type[0]

    try:
        if system_type == 'linear':
            from ETCetera.util.construct_from_file_linearPETC import \
                construct_linearPETC_traffic_from_file
            traffic = construct_linearPETC_traffic_from_file(input_file)

        if system_type in ['general', 'nonlinear', 'non-linear']:
            from ETCetera.util.construct_from_file_nonlinearETC import \
                construct_nonlinearETC_traffic_from_file
            traffic = construct_nonlinearETC_traffic_from_file(input_file)
            traffic.visualize()
        # More to follow...

    except SentientBaseException as e:
        print(str(e))
        sys.exit()
    else:
        if args.output_file:
            from config import save_path
            if not args.output_file.startswith(save_path):
                args.output_file = os.path.join(save_path, args.output_file)

            traffic.export(args.output_file, out_type)


import logging
import socket
import time

logging.basicConfig(format=('[%(asctime)s] %(levelname)s-%(name)s-%(message)s'),
                    level=logging.INFO)


def split_result(result):
    return [x for x in result.split("\n") if x != "pnet # "]


def has_ok_response(result):
    if "ok" in result:
        return True
    return False


def check_for_prompt(result):
    """
    Checks the results of the TCP command for prompt. The prompt signifies
    that the system is ready to accept another command.

    :param result: The string that we're going to check for the prompt
    :returns True if we found the result, False otherwise.
    """
    return result.find("pnet # ") != -1


def retrieve_response(conn):
    result = ""
    while not check_for_prompt(result):
        result = result + conn.recv(1024).decode('ascii')
    return split_result(result)


def parse_startup_message(conn):
    """
    Logs the startup information for the socket.

    :param conn: The connection to use to get the information.
    """
    result = ""

    logging.info('Getting initial information from the connection...')
    while not check_for_prompt(result):
        result = conn.recv(1024).decode('ascii')
    for line in split_result(result):
        logging.info(line)
    logging.info("Finished parsing initial startup information.")


def get_parameter_report(conn):
    logging.info('Sending a request to report the status of the system...')
    conn.send('report\n'.encode())
    logging.info("Receiving results of report")
    return retrieve_response(conn)


def set_parameter(conn, par, val):
    """
    Handles setting parameters
    """
    logging.info('Sending a request to set a parameter.')
    conn.send(f'set {par} {val}\n'.encode())
    result = retrieve_response(conn)
    logging.info(f"Received the following reply: '{result}'")

    if not has_ok_response(result):
        if result[0].find("not found") != -1:
            raise AttributeError(f"Attempted to set a parameter ({par}) that doesn't exist!")
        if result[0].find("no seting/value pair") != -1:
            raise AttributeError(f'The set command requires a setting/value pair to operate!')


def execute_run_control(conn, cmd):
    """
    Executes commands related to executing runs with the system.
    :param conn: The connection to use
    :param cmd: The sub-command to execute.
    """
    logging.info(f'Executing run control operation: {cmd}')
    conn.send(f'run {cmd}\n'.encode())
    result = retrieve_response(conn)
    logging.info(f"Received the following reply: '{result}")

    if not has_ok_response(result) and cmd:
        raise RuntimeError(f"Executing run control command ended with error: {result}")

def program_system(conn):
    logging.info("Loading parameter changes to module before run.")
    conn.send(b'program\n')
    result = retrieve_response(conn)
    if has_ok_response(result):
        logging.info("Finished loading parameters, starting run.")
    else:
        raise RuntimeError(f"Loading parameters failed with message: {result}")


def main(conn):
    """
    The main driver for our script
    :param conn: The connection that we'll be using for all of our operations.
    """
    parse_startup_message(conn)
    report = get_parameter_report(conn)

    try:
        set_parameter(conn, "UDP_PAUSE", 10)
        set_parameter(conn, "UDP_PAUSEer", 10)
    except AttributeError as err:
        logging.error(str(err))

    try:
        program_system(conn)
        execute_run_control(conn, "start")
        start_time = time.time()
        while time.time() - start_time <= 5:
            execute_run_control(conn, "")
            time.sleep(1)
        execute_run_control(conn, "stop")
    except RuntimeError as err:
        logging.error(str(err))

    conn.close()


if __name__ == '__main__':
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(('192.168.122.54', 31057))
        main(s)
    except KeyboardInterrupt:
        logging.info("Received keyboard interrupt. Stopping execution.")
    except Exception as e:
        logging.exception(e)

# Copyright (c) 2019 XIA LLC
# All rights reserved.
#
# Redistribution and use in source and binary forms,
# with or without modification, are permitted provided
# that the following conditions are met:
#
#   1. Redistributions of source code must retain the above
#        copyright notice, this list of conditions and the
#        following disclaimer.
#   2. Redistributions in binary form must reproduce the
#        above copyright notice, this list of conditions and the
#        following disclaimer in the documentation and/or other
#        materials provided with the distribution.
#   3. Neither the name of XIA LLC
#        nor the names of its contributors may be used to endorse
#        or promote products derived from this software without
#        specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
# TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
# THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
import logging
import socket
import time

logging.basicConfig(format=('[%(asctime)s] %(levelname)s-%(name)s-%(message)s'),
                    level=logging.INFO)


def split_result(result):
    """
    Splits out the results based on new line characters (\n) from the TCP response and removes the
    prompt.
    :param result: The result that we're going to process.
    :return: A list containing each line of the response.
    """
    return [x for x in result.split("\n") if x != "pnet # "]


def has_ok_response(result):
    """
    Verifies that the "ok" for a successful command execution exists in the results.
    :param result: The result list that we're going to search in
    :return: True if the command finished with a success. False otherwise.
    """
    if "ok" in result:
        return True
    return False


def check_for_prompt(result):
    """
    Checks the results of the TCP command for prompt. The prompt signifies
    that the system is ready to accept another command.

    :param result: The string that we're going to check for the prompt
    :return: True if we found the result, False otherwise.
    """
    return result.find("pnet # ") != -1


def retrieve_response(conn):
    """
    Retrieves the full response from a TCP request. The response is considered complete when we've
    hit the prompt ("pnet # "). This indicates that the system is ready to accept another command.
    :param conn: The connection to parse for the response.
    :return: A list containing each line of the response.
    """
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
    """
    Gets the parameter report from the system as a list. Each element corresponds to one and
    only one parameter.
    :param conn: The connection that we'll request the report from.
    :return: A list containing each element of the current system settings.
    """
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
        set_parameter(conn, "DUMMY_PARAMETER", 8675309)
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

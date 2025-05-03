#
# Created on Sat May 03 2025
# Author: Dimetri Chau
# https://github.com/Deonixlive
#
# Copyright (c) 2025 Dimetri Chau
#


# General
fatal_error = {
    "log_info": "\033[91m[FATAL ERROR]\033[0m {0}",
    "console": "\033[91m[FATAL ERROR]\033[0m {0}"
}

# MSP messages
msp_service_init = {
    "log_info": "\033[93m[MSP_SERVICE INITIATED]\033[0m with parameters: port: {0}, baud_rate: {1}",
    "console": "\033[93m[MSP_SERVICE INITIATED]\033[0m with parameters: port: {0}, baud_rate: {1}"
    }

msp_service_connected = {
    "log_info": "\033[92m[MSP_SERVICE CONNECTED]\033[0m",
    "console": "\033[92m[MSP_SERVICE CONNECTED]\033[0m"
    }

msp_service_disconnected = {
    "log_info": "\033[91m[MSP_SERVICE DISCONNECTED]\033[0m",
    "console": "\033[91m[MSP_SERVICE DISCONNECTED]\033[0m"
    }

msp_not_connected = {
    "log_info": "[MSP_SERVICE] coult not connect to FC",
    "console": "\033[91m[MSP_SERVICE]\033[0m coult not connect to FC"
    }


msp_service_rebooting = {
    "log_info": "\033[93m[MSP_SERVICE REBOOTING]\033[0m",
    "console": "\033[93m[MSP_SERVICE REBOOTING]\033[0m"
    }

msp_service_command_submitted = {
    "log_info": "\033[93m[MSP_SERVICE COMMAND SUBMITTED]\033[0m: {0}",
    "console": "\033[93m[MSP_SERVICE COMMAND SUBMITTED]\033[0m: {0}"
    }

msp_service_execution_loop_started = {
    "log_info": "\033[MSP_SERVICE EXEC LOOP STARTED]\033[0m",
    "console": "\033[92m[MSP_SERVICE EXEC LOOP STARTED]\033[0m"
}

msp_service_execute_command = {
    "log_info": "\033[93m[MSP_SERVICE EXECUTING COMMAND]\033[0m: with id {0} and data: {1}",
    "console": "\033[93m[MSP_SERVICE EXECUTING COMMAND]\033[0m: with id {0} and data: {1}"
    }

msp_service_command_execution_failed = {
    "log_info": "\033[91m[MSP_SERVICE COMMAND EXECUTION FAILED]\033[0m: with cmd: {0} and error: {1}",
    "console": "\033[91m[MSP_SERVICE COMMAND EXECUTION FAILED]\033[0m: with cmd:  {0} and error: {1}"
    }


# Telemetry messages
telemetry_msp_service_init = {
    "log_info": "[TELEMETRY]",
    "console": "\033[93m[TELEMETRY INITIATED]\033[0m with parameters: port: {0}, baud_rate: {1}"
    }

telemetry_process_connected = {
    "log_info": "Telemetry: connected with '{0}'",
    "console": "Telemetry: connected with \033[92m{0}\033[0m"
    }

telemetry_service_execute_update = {
    "log_info": "\033[93m[TELEMETRY_SERVICE EXECUTING UPDATE]\033[0m: with name {0}",
    "console": "\033[93m[TELEMETRY_SERVICE EXECUTING UPDATE]\033[0m: with name {0}"
    }

telemetry_update_function_not_callable = {
    "log_info": "\033[91m[TELEMETRY_SERVICE UPDATE FUNCTION NOT CALLABLE]\033[0m: with name {0}",
    "console": "\033[91m[TELEMETRY_SERVICE UPDATE FUNCTION NOT CALLABLE]\033[0m: with name {0}"
    }

telemetry_log_copter_state = {
    "log_debug": "\033[93m[TELEMETRY]\033[0m Copter State: {0}",
    "console": "\033[93m[TELEMETRY]\033[0m Copter State: {0}"
    }

telemetry_log_frequency = {
    "log_info": "\033[93m[TELEMETRY LOG FREQUENCY]\033[0m: {0} Hz",
    "console": "\033[93m[TELEMETRY LOG FREQUENCY]\033[0m: {0} Hz"
    }

# Copter
copter_msp_not_ready = {
    "log_info": "\033[93m[COPTER]\033[0m: MSP not ready",
    "console": "\033[93m[COPTER]\033[0m: MSP not ready"
}

copter_stopping_threads = {
    "log_info": "[STOPPING THREADS]",
    "console": "\033[93m[STOPPING THREADS]\033[0m"
    }


def display(msg, params=[]):
    # if msg.get('log_info'):
    #     logger.log_message(None, msg['log_info'].format(*params), 'info')
    # if msg.get('log_debug'):
    #     logger.log_message(None, msg['log_debug'].format(*params), 'debug')
    # if msg.get('log_fatal'):
    #     logger.log_message(None, msg['log_fatal'].format(*params), 'fatal')
    if msg.get('console'):
        print(msg['console'].format(*params))
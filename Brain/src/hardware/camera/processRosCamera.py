# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""ROS2 RealSense 카메라 프로세스.

기존 `processCamera.py`(picamera2 전용)를 건드리지 않고, RealSense + ROS2
이미지 토픽을 받아 대시보드로 전달하는 별도 프로세스입니다.
"""

if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

import time

from src.templates.workerprocess import WorkerProcess
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.allMessages import StateChange
from src.statemachine.systemMode import SystemMode
from src.hardware.camera.threads.threadRosCamera import RosCameraThread


class processRosCamera(WorkerProcess):
    """RealSense ROS 카메라 프로세스."""

    def __init__(self, queueList, logging, ready_event=None, debugging: bool = False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.stateChangeSubscriber = messageHandlerSubscriber(
            self.queuesList, StateChange, "lastOnly", True
        )
        super(processRosCamera, self).__init__(self.queuesList, ready_event)

    # ===================================== INIT TH ====================================
    def _init_threads(self):
        cam_thread = RosCameraThread(
            self.queuesList,
            self.logging,
            debugging=self.debugging,
        )
        self.threads.append(cam_thread)

    # ================================ STATE CHANGE ====================================
    def state_change_handler(self):
        message = self.stateChangeSubscriber.receive()
        if message is not None:
            modeDict = SystemMode[message].value["camera"]["process"]
            if modeDict.get("enabled", True):
                self.resume_threads()
            else:
                self.pause_threads()


if __name__ == "__main__":
    from multiprocessing import Queue
    import logging

    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
    }

    logger = logging.getLogger()
    process = processRosCamera(queueList, logger, debugging=True)
    process.daemon = True
    process.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        process.stop()

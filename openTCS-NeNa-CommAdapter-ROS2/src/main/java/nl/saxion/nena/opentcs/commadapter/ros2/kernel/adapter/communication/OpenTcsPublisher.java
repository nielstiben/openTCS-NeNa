package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;

/* Copyright 2017 Esteve Fernandez <esteve@apache.org>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

import java.util.concurrent.TimeUnit;

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.concurrent.Callback;
import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.timer.WallTimer;

public class OpenTcsPublisher extends BaseComposableNode {
    private int count;

    private Publisher<std_msgs.msg.String> publisher;

    private WallTimer timer;

    public OpenTcsPublisher() {
        super("OpenTCS_publisher");
        this.count = 0;
        // Publishers are type safe, make sure to pass the message type
        this.publisher = node.<std_msgs.msg.String>createPublisher(std_msgs.msg.String.class, "topic");
        Callback timerCallback = () -> {
            std_msgs.msg.String message = new std_msgs.msg.String();
            message.setData("Hello, world! " + this.count);
            this.count++;
            System.out.println("Publishing: [" + message.getData() + "]");
            this.publisher.publish(message);
        };
        this.timer = node.createWallTimer(500, TimeUnit.MILLISECONDS, timerCallback);
    }

    public static void main(String[] args) throws InterruptedException {
        // Initialize RCL
        RCLJava.rclJavaInit();
        RCLJava.spin(new OpenTcsPublisher());
    }
}
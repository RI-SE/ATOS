```mermaid
 graph LR
    subgraph ROS2
        subgraph RCLNodeJS
            subgraph NodeJS
                ConfigPanel
                ControlPanel
            end
        end
    end
    subgraph ROS2
        ATOS
    end
    subgraph Client
        WebBrowser
    end


    WebBrowser-->|Request website :3000|NodeJS
    NodeJS-->|Serve website :3000|WebBrowser
    WebBrowser<-->|ws :8081|NodeJS


    ControlPanel<-->|ROS_messages|ATOS
    ConfigPanel<-->|ROS_messages|ATOS
```

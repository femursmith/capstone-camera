graph LR
    subgraph Device Area
        A[Motion Detected<br>(HC-SR501)] --> B(Capture Image<br>[XIAO ESP32S3]);
        B --> C{Send Image +<br>Camera/User IDs};
    end

    subgraph Server Area (Node.js)
        D[Receive Data] --> E{Authenticate & Upload Image};
        E --> F[Store Event Metadata<br>(Timestamp, File ID, etc.)];
        F --> G{Publish Event Details};
    end

    subgraph Cloud & Services
        H[(Google Drive)]
        I[(Database<br>MongoDB)]
        J[(MQTT Broker<br>e.g., EMQX)]
    end

    subgraph User Area
        K{Receive Notification};
        K --> L{Request Image<br>(via Server)};
        L --> M[Display Image];
    end

    %% Connections
    C -- HTTP POST --> D;
    E -- Google Drive API --> H;
    H -- File ID --> E;
    F -- DB Write --> I;
    G -- MQTT Publish --> J;
    J -- MQTT Subscribe --> K;
    L -- HTTP GET --> D; %% Assuming server handles fetch request
    D -- Fetch Image --> H; %% Server fetches from GDrive
    H -- Image Data --> D; %% GDrive returns image to server
    D -- Image Data --> M; %% Server sends image to client

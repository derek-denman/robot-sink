import React from "react";
import ReactDOM from "react-dom/client";
import { ConfigProvider, theme } from "antd";
import App from "./App";
import "antd/dist/reset.css";
import "./styles.css";

ReactDOM.createRoot(document.getElementById("root") as HTMLElement).render(
  <React.StrictMode>
    <ConfigProvider
      theme={{
        algorithm: theme.darkAlgorithm,
        token: {
          colorPrimary: "#36cfc9",
          colorInfo: "#4d9fff",
          borderRadius: 8,
          fontFamily: "'IBM Plex Sans', 'Segoe UI', sans-serif"
        }
      }}
    >
      <App />
    </ConfigProvider>
  </React.StrictMode>,
);

import { Layout, Menu, theme } from "antd";
import "./App.css";
import { Outlet, useNavigate } from "react-router-dom";
import { menuPoints } from "./ui/header/nav";
import Sider from "antd/es/layout/Sider";

const { Header, Content, Footer } = Layout;

const AppLayout: React.FC = () => {
  const {
    token: { colorBgContainer, borderRadiusLG },
  } = theme.useToken();

  const navigate = useNavigate();

  return (
    <Layout style={{ height: "100%" }}>
      <Sider collapsible>
        <Menu
          theme="dark"
          defaultSelectedKeys={["2"]}
          items={menuPoints}
          mode="inline"
          onClick={(info) => navigate(info.key)}
          style={{ flex: 1, minWidth: 0 }}
        />
      </Sider>
      <Content style={{ flex: 1, padding: 24, height: "100%" }}>
        <div
          style={{
            background: colorBgContainer,
            padding: 24,
            borderRadius: borderRadiusLG,
            height: "100%",
          }}
        >
          <Outlet />
        </div>
      </Content>
    </Layout>
  );
};

export default AppLayout;

import { Layout, Menu, theme } from "antd";
import "./App.css";
import { Outlet, useNavigate } from "react-router-dom";
import { menuPoints } from "./ui/header/nav";

const { Header, Content, Footer } = Layout;

const AppLayout: React.FC = () => {
  const {
    token: { colorBgContainer, borderRadiusLG },
  } = theme.useToken();

  const navigate = useNavigate();

  return (
    <Layout
      style={{ height: "100%", display: "flex", flexDirection: "column" }}
    >
      <Header style={{ display: "flex", alignItems: "center" }}>
        <div className="demo-logo" />
        <Menu
          theme="dark"
          mode="horizontal"
          defaultSelectedKeys={["2"]}
          items={menuPoints}
          onClick={(info) => navigate(info.key)}
          style={{ flex: 1, minWidth: 0 }}
        />
      </Header>
      <Content style={{ flex: 1, padding: "0 48px", height: '100%' }}>
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
      <Footer style={{ textAlign: "center" }}>
        УлГТУ ©{new Date().getFullYear()}
      </Footer>
    </Layout>
  );
};

export default AppLayout;

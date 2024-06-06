import { Button, Card, Form, Input, Layout } from "antd";
import { useCallback } from "react";
import { useAuthStore } from "../../../store/useAuthStore";
import { apiClientClass } from "../../../shared/api";
import { ApiConfig } from "../../../shared/api/http-client";
import "./Login.scss";
import { useNavigate } from "react-router-dom";

interface Login {
  username: string;
  password: string;
}

export const Login = () => {
  const { login } = useAuthStore();

  const configMcc: ApiConfig = {
    baseUrl: "http://localhost:8000",
  };

  const apiClient = new apiClientClass(configMcc);

  const navigate = useNavigate();

  const authorize = useCallback((data: Login) => {
    apiClient.Token.tokenCreate({
      username: data.username,
      password: data.password,
    }).then(({data}) => login(data.access, data.user_id));
  }, []);

  return (
    <Layout style={{ height: "100vh", overflow: "auto" }} className="layout">
      <Card className="login-card">
        <Form onFinish={authorize} layout="vertical">
          <Form.Item name="username" label="Имя пользователя">
            <Input />
          </Form.Item>
          <Form.Item name="password" label="Пароль">
            <Input type="password" />
          </Form.Item>
          <Form.Item>
            <Button type="primary" htmlType="submit" className="auth-btn">
              Войти
            </Button>
          </Form.Item>
          <Form.Item>
            <Button type="default" className="auth-btn" onClick={() => navigate('/register')}>Регистрация</Button>
          </Form.Item>
        </Form>
      </Card>
    </Layout>
  );
};

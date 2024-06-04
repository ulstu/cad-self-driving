import { Button, Card, Form, Input, Layout } from "antd";
import { useCallback } from "react";
import { apiClientClass } from "../../../shared/api";
import { ApiConfig } from "../../../shared/api/http-client";
import { useNavigate } from "react-router-dom";
import "./Register.scss";

interface Register {
  username: string;
  password: string;
  first_name: string;
  last_name: string;
  email: string;
}

export const Register = () => {
  const configMcc: ApiConfig = {
    baseUrl: "http://localhost:8000",
  };

  const apiClient = new apiClientClass(configMcc);

  const navigate = useNavigate();

  const userCreate = useCallback((data: Register) => {
    apiClient.Users.usersCreate({ ...data, is_superuser: false }).then(() => navigate("/login"));
  }, []);

  return (
    <Layout style={{ height: "100vh", overflow: "auto" }} className="layout">
      <Card className="register-card">
        <Form onFinish={userCreate} layout="vertical">
          <Form.Item
            name="username"
            label="Имя пользователя"
            rules={[{ required: true }]}
          >
            <Input />
          </Form.Item>
          <Form.Item
            name="password"
            label="Пароль"
            rules={[{ required: true }]}
          >
            <Input type="password" />
          </Form.Item>
          <Form.Item name="first_name" label="Имя" rules={[{ required: true }]}>
            <Input type="text" />
          </Form.Item>
          <Form.Item
            name="last_name"
            label="Фамилия"
            rules={[{ required: true }]}
          >
            <Input type="text" />
          </Form.Item>
          <Form.Item
            name="email"
            label="Адрес электронной почты"
            rules={[{ required: true }]}
          >
            <Input type="email" />
          </Form.Item>
          <Form.Item>
            <Button type="primary" htmlType="submit" style={{ width: "100%" }}>
              Зарегестрироваться
            </Button>
          </Form.Item>
        </Form>
      </Card>
    </Layout>
  );
};

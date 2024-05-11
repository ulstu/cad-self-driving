import { Button, Form, Input, Layout } from "antd";
import { useCallback } from "react";
import { useAuthStore } from "./store/useAuthStore";
import { apiClientClass } from "./shared/api";
import { ApiConfig } from "./shared/api/http-client";

interface Login {
  username: string;
  password: string;
}

export const Login = () => {
  const { login } = useAuthStore();

  const configMcc: ApiConfig = {
    baseUrl: "http://localhost:8000",
    baseApiParams: {
      headers: {
        Authorization: `Bearer ${useAuthStore((state) => state.token)}`,
      },
    },
  };

  const apiClient = new apiClientClass(configMcc);

  const authorize = useCallback((data: Login) => {
    apiClient.Token.tokenCreate({
      username: data.username,
      password: data.password,
      access: "",
      refresh: "",
    }).then((data) => login(data.data.access));
  }, []);

  return (
    <Layout>
      <Form onFinish={authorize}>
        <Form.Item name="username">
          <Input />
        </Form.Item>
        <Form.Item name="password">
          <Input />
        </Form.Item>
        <Button type="primary" htmlType="submit">
          Войти
        </Button>
      </Form>
    </Layout>
  );
};

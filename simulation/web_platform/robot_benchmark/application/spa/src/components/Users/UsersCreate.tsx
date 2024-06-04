import { Modal, Form, Input, Checkbox } from "antd";

export const UsersCreate = ({ visible, onCreate, onCancel }) => {
  const [form] = Form.useForm();

  return (
    <Modal
      open={visible}
      title="Создать пользователя"
      okText="Создать"
      cancelText="Закрыть"
      onCancel={onCancel}
      onOk={() => {
        form
          .validateFields()
          .then((values) => {
            form.resetFields();
            onCreate(values);
          })
          .catch((info) => {
            console.log("Validate Failed:", info);
          });
      }}
    >
      <Form form={form} layout="vertical" name="form_in_modal">
        <Form.Item
          name="username"
          label="Логин"
          rules={[
            {
              required: true,
              message: "Please input the name of the tournament!",
            },
            { max: 255, message: "Name must be 255 characters or less" },
          ]}
        >
          <Input />
        </Form.Item>
        <Form.Item
          name="password"
          label="Пароль"
          rules={[
            {
              required: true,
              message: "Please input the description of the tournament!",
            },
            {
              max: 5000,
              message: "Description must be 5000 characters or less",
            },
          ]}
        >
          <Input type="password" />
        </Form.Item>
        <Form.Item
          name="first_name"
          label="Имя"
          rules={[
            {
              required: true,
              message: "Please select the start date and time!",
            },
          ]}
        >
          <Input type="text" />
        </Form.Item>
        <Form.Item
          name="last_name"
          label="Фамилия"
          rules={[
            { required: true, message: "Please select the end date and time!" },
          ]}
        >
          <Input type="text" />
        </Form.Item>
        <Form.Item
          label="Адрес электронной почты"
          name="email"
          rules={[{ required: true, message: "Please input!" }]}
        >
          <Input type="email" />
        </Form.Item>
        <Form.Item label="Является администратором" name="is_superuser">
          <Checkbox defaultChecked={false} value={false} />
        </Form.Item>
      </Form>
    </Modal>
  );
};

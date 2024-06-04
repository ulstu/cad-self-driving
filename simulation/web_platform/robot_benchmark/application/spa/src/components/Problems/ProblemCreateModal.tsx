import { Modal, Form, Input, Select, Button, InputNumber } from 'antd';
import { WorldPathEnum } from '../../shared/api'; // Assuming you have exported WorldPathEnum

const ProblemCreateModal = ({ visible, onCreate, onCancel }) => {
  const [form] = Form.useForm();

  const onFormSubmit = () => {
    form.validateFields()
      .then(values => {
        form.resetFields();
        onCreate(values);
      })
      .catch(info => {
        console.log('Validate Failed:', info);
      });
  };

  return (
    <Modal
      title="Create New Problem"
      visible={visible}
      onCancel={onCancel}
      footer={[
        <Button key="back" onClick={onCancel}>
          Cancel
        </Button>,
        <Button key="submit" type="primary" onClick={onFormSubmit}>
          Create
        </Button>,
      ]}
    >
      <Form
        form={form}
        layout="vertical"
        name="create_problem_form"
        initialValues={{ modifier: 'public' }}
      >
        <Form.Item
          name="title"
          label="Title"
          rules={[{ required: true, message: 'Please input the title!' }, { max: 300 }]}
        >
          <Input />
        </Form.Item>
        <Form.Item
          name="description"
          label="Description"
          rules={[{ max: 1000 }]}
        >
          <Input.TextArea rows={4} />
        </Form.Item>
        <Form.Item
          name="world_path"
          label="World Path"
          rules={[{ required: true, message: 'Please select the world path!' }]}
        >
          <Select placeholder="Select a world path">
            {Object.keys(WorldPathEnum).map(key => (
              <Select.Option key={key} value={WorldPathEnum[key]}>
                {key}
              </Select.Option>
            ))}
          </Select>
        </Form.Item>
        <Form.Item
          name="difficulty"
          label="Difficulty"
          rules={[{ required: true, message: 'Please input the difficulty!' }]}
        >
          <InputNumber min={1} max={10} step={0.1} />
        </Form.Item>
        <Form.Item
          name="image"
          label="Image URL"
        >
          <Input />
        </Form.Item>
      </Form>
    </Modal>
  );
};

export default ProblemCreateModal;

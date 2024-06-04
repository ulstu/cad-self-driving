import { useCallback, useEffect, useState } from "react";
import { apiClientClass } from "../../shared/api";
import { ApiConfig } from "../../shared/api/http-client";
import { useAuthStore } from "../../store/useAuthStore";
import { User } from "../../shared/api/data-contracts";
import { Button, FloatButton, Space, Table, Tooltip } from "antd";
import { DeleteOutlined, EditOutlined, PlusOutlined } from "@ant-design/icons";
import TournamentList from "../Tournament/TournamentList";
import { UsersCreate } from "./UsersCreate";
import { UsersEdit } from "./UsersEdit";

export const UsersList = () => {
  const [users, setUsers] = useState<User[]>([]);
  const [editData, setEditData] = useState<User>();
  const [showUserCreate, setShowUserCreate] = useState<boolean>(false);
  const [showUserEdit, setShowUserEdit] = useState<number | boolean>(false);
  const { token } = useAuthStore();

  const configMcc: ApiConfig = {
    baseUrl: "http://localhost:8000",
    baseApiParams: {
      headers: {
        Authorization: `Bearer ${token}`,
      },
    },
  };

  const apiClient = new apiClientClass(configMcc);

  const fetchUsers = useCallback(() => {
    apiClient.Users.usersList().then(({ data }) => setUsers(data));
  }, []);

  useEffect(() => {
    fetchUsers();
  }, [fetchUsers]);

  const usersTableColumns = [
    {
      title: "",
      dataIndex: "id",
      key: "id",
      render: (id) => (
        <Space>
          <Tooltip title="Редактировать пользователя">
            <Button
              type="default"
              shape="circle"
              icon={<EditOutlined />}
              onClick={() => {
                onEditClick(id);
                setShowUserEdit(true);
              }}
            />
          </Tooltip>
          <Tooltip title="Удалить пользователя">
            <Button type="default" shape="circle" icon={<DeleteOutlined />} />
          </Tooltip>
        </Space>
      ),
    },
    {
      title: "Логин",
      dataIndex: "username",
      key: "username",
    },
    {
      title: "Имя",
      dataIndex: "first_name",
      key: "first_name",
    },
    {
      title: "Фамилия",
      dataIndex: "last_name",
      key: "last_name",
    },
    {
      title: "Почта",
      dataIndex: "email",
      key: "email",
    },
    {
      title: "Является админом",
      dataIndex: "is_superuser",
      key: "is_superuser",
    },
  ];

  const onCreate = (values: User) => {
    apiClient.Users.usersCreate({
      ...values,
      is_superuser: values.is_superuser ?? false,
    }).then(() => {
      setShowUserCreate(false);
      fetchUsers();
    });
  };

  const onEditClick = (id: number) => {
    apiClient.Users.usersRetrieve(id)
      .then(({ data }) => setEditData(data))
      .then(() => setShowUserEdit(id));
  };

  const editUser = (data: User) => {
    apiClient.Users.usersUpdate(showUserEdit, data).then(() => fetchUsers());
  };

  return (
    <div>
      <h1>Пользователи</h1>
      <UsersCreate
        visible={showUserCreate}
        onCreate={onCreate}
        onCancel={() => setShowUserCreate(false)}
      />
      <UsersEdit
        visible={showUserEdit}
        onEdit={(data) => editUser(data)}
        data={editData}
        onCancel={() => setShowUserEdit(false)}
      />
      <Table columns={usersTableColumns} dataSource={users} />
      <FloatButton
        shape="square"
        tooltip={<>Создать пользователя</>}
        type="primary"
        style={{ right: 42 }}
        onClick={() => setShowUserCreate(true)}
        icon={<PlusOutlined />}
      />
    </div>
  );
};

export default TournamentList;

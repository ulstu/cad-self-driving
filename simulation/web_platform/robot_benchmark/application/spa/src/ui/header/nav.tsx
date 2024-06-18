import { FileTextOutlined, TrophyOutlined, UserOutlined } from "@ant-design/icons";
import { MenuProps } from "antd";

type MenuItem = Required<MenuProps>['items'][number];

const adminPoints: MenuItem[] = [{
  key: '/users',
  label: 'Пользователи',
  icon: <UserOutlined />
}];

export const menuPoints: MenuItem[] = [
  {
    key: "/leaderboard",
    label: "Турнирная таблица",
  },
  {
    key: "/problems",
    label: "Задачи",
    icon: <FileTextOutlined />,
  },
  {
    key: "/tournaments",
    label: "Турниры",
    icon: <TrophyOutlined />,
  },
  ...adminPoints
];

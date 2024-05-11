/* eslint-disable */
/* tslint:disable */
/*
 * ---------------------------------------------------------------
 * ## THIS FILE WAS GENERATED VIA SWAGGER-TYPESCRIPT-API        ##
 * ##                                                           ##
 * ## AUTHOR: acacode                                           ##
 * ## SOURCE: https://github.com/acacode/swagger-typescript-api ##
 * ---------------------------------------------------------------
 */

/** Сериализатор для модель соревнования */
export interface PatchedProblem {
    id?: number;
    /** @maxLength 300 */
    title?: string;
    /** @maxLength 1000 */
    description?: string | null;
    /**
     * * `webots_ros2_suv` - webots_ros2_suv
     * * `webots_ros2_tesla` - webots_ros2_tesla
     * * `webots_ros2_control` - webots_ros2_control
     * * `webots_ros2_driver` - webots_ros2_driver
     * * `webots_ros2_epuck` - webots_ros2_epuck
     * * `webots_ros2_importer` - webots_ros2_importer
     * * `webots_ros2_mavic` - webots_ros2_mavic
     * * `webots_ros2_msgs` - webots_ros2_msgs
     * * `webots_ros2_tests` - webots_ros2_tests
     * * `webots_ros2_tiago` - webots_ros2_tiago
     * * `webots_ros2_turtlebot` - webots_ros2_turtlebot
     * * `webots_ros2_universal_robot` - webots_ros2_universal_robot
     */
    world_path?: WorldPathEnum;
    /** @format uri */
    image?: string | null;
    /** @format double */
    difficulty?: number;
    author?: number;
    users?: number[];
}

/** Сериализатор для модели many-to-many Соревнования-Пользователи */
export interface PatchedProblemUser {
    id?: number;
    is_completed?: boolean;
    /**
     * @min -2147483648
     * @max 2147483647
     */
    points?: number;
    robot_panel_port?: number;
    vs_port?: number;
    webots_stream_port?: number;
    user?: number;
    problem?: number;
}

/** Сериализатор для модель соревнования */
export interface PatchedTournament {
    id?: number;
    /** @maxLength 255 */
    name?: string;
    /** @maxLength 5000 */
    description?: string;
    /** @format date-time */
    date_start?: Date;
    /** @format date-time */
    date_end?: Date;
    problems?: number[];
    users?: number[];
}

/** Сериализатор для модель many-to-many Соревнования-Пользователи */
export interface PatchedTournamentUser {
    id?: number;
    is_completed?: boolean;
    /**
     * @min -2147483648
     * @max 2147483647
     */
    points?: number;
    user?: number;
    tournament?: number;
}

export interface PatchedUser {
    id?: number;
    /** @maxLength 128 */
    password?: string;
    /**
     * Required. 150 characters or fewer. Letters, digits and @/./+/-/_ only.
     * @maxLength 150
     * @pattern ^[\w.@+-]+$
     */
    username?: string;
    /** @maxLength 150 */
    first_name?: string;
    /** @maxLength 150 */
    last_name?: string;
    /**
     * Email address
     * @format email
     * @maxLength 254
     */
    email?: string;
}

/** Сериализатор для модель соревнования */
export interface Problem {
    id: number;
    /** @maxLength 300 */
    title: string;
    /** @maxLength 1000 */
    description?: string | null;
    /**
     * * `webots_ros2_suv` - webots_ros2_suv
     * * `webots_ros2_tesla` - webots_ros2_tesla
     * * `webots_ros2_control` - webots_ros2_control
     * * `webots_ros2_driver` - webots_ros2_driver
     * * `webots_ros2_epuck` - webots_ros2_epuck
     * * `webots_ros2_importer` - webots_ros2_importer
     * * `webots_ros2_mavic` - webots_ros2_mavic
     * * `webots_ros2_msgs` - webots_ros2_msgs
     * * `webots_ros2_tests` - webots_ros2_tests
     * * `webots_ros2_tiago` - webots_ros2_tiago
     * * `webots_ros2_turtlebot` - webots_ros2_turtlebot
     * * `webots_ros2_universal_robot` - webots_ros2_universal_robot
     */
    world_path: WorldPathEnum;
    /** @format uri */
    image?: string | null;
    /** @format double */
    difficulty: number;
    author: number;
    users: number[];
}

/** Сериализатор для модели many-to-many Соревнования-Пользователи */
export interface ProblemUser {
    id: number;
    is_completed?: boolean;
    /**
     * @min -2147483648
     * @max 2147483647
     */
    points?: number;
    robot_panel_port: number;
    vs_port: number;
    webots_stream_port: number;
    user: number;
    problem: number;
}

export interface TokenObtainPair {
    username: string;
    password: string;
    access: string;
    refresh: string;
}

export interface TokenRefresh {
    access: string;
    refresh: string;
}

/** Сериализатор для модель соревнования */
export interface Tournament {
    id: number;
    /** @maxLength 255 */
    name: string;
    /** @maxLength 5000 */
    description: string;
    /** @format date-time */
    date_start: Date;
    /** @format date-time */
    date_end: Date;
    problems: number[];
    users: number[];
}

/** Сериализатор для модель many-to-many Соревнования-Пользователи */
export interface TournamentUser {
    id: number;
    is_completed?: boolean;
    /**
     * @min -2147483648
     * @max 2147483647
     */
    points?: number;
    user: number;
    tournament: number;
}

export interface User {
    id: number;
    /** @maxLength 128 */
    password: string;
    /**
     * Required. 150 characters or fewer. Letters, digits and @/./+/-/_ only.
     * @maxLength 150
     * @pattern ^[\w.@+-]+$
     */
    username: string;
    /** @maxLength 150 */
    first_name?: string;
    /** @maxLength 150 */
    last_name?: string;
    /**
     * Email address
     * @format email
     * @maxLength 254
     */
    email?: string;
}

/**
 * * `webots_ros2_suv` - webots_ros2_suv
 * * `webots_ros2_tesla` - webots_ros2_tesla
 * * `webots_ros2_control` - webots_ros2_control
 * * `webots_ros2_driver` - webots_ros2_driver
 * * `webots_ros2_epuck` - webots_ros2_epuck
 * * `webots_ros2_importer` - webots_ros2_importer
 * * `webots_ros2_mavic` - webots_ros2_mavic
 * * `webots_ros2_msgs` - webots_ros2_msgs
 * * `webots_ros2_tests` - webots_ros2_tests
 * * `webots_ros2_tiago` - webots_ros2_tiago
 * * `webots_ros2_turtlebot` - webots_ros2_turtlebot
 * * `webots_ros2_universal_robot` - webots_ros2_universal_robot
 */
export enum WorldPathEnum {
    WebotsRos2Suv = "webots_ros2_suv",
    WebotsRos2Tesla = "webots_ros2_tesla",
    WebotsRos2Control = "webots_ros2_control",
    WebotsRos2Driver = "webots_ros2_driver",
    WebotsRos2Epuck = "webots_ros2_epuck",
    WebotsRos2Importer = "webots_ros2_importer",
    WebotsRos2Mavic = "webots_ros2_mavic",
    WebotsRos2Msgs = "webots_ros2_msgs",
    WebotsRos2Tests = "webots_ros2_tests",
    WebotsRos2Tiago = "webots_ros2_tiago",
    WebotsRos2Turtlebot = "webots_ros2_turtlebot",
    WebotsRos2UniversalRobot = "webots_ros2_universal_robot",
}

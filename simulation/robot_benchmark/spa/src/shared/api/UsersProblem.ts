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

import { PatchedProblemUser, ProblemUser } from "./data-contracts";
import { ContentType, HttpClient, RequestParams } from "./http-client";

export class UsersProblem<SecurityDataType = unknown> extends HttpClient<SecurityDataType> {
    /**
     * No description
     *
     * @tags users-problem
     * @name UsersProblemList
     * @summary Детальная информация о всех задачах пользователей
     * @request GET:/api/users-problem/
     * @secure
     */
    usersProblemList = (
        query?: {
            /** Which field to use when ordering the results. */
            ordering?: string;
        },
        params: RequestParams = {},
    ) =>
        this.request<ProblemUser[], any>({
            path: `/api/users-problem/`,
            method: "GET",
            query: query,
            secure: true,
            format: "json",
            ...params,
        });
    /**
     * No description
     *
     * @tags users-problem
     * @name UsersProblemCreate
     * @summary Создание задачах пользователю
     * @request POST:/api/users-problem/
     * @secure
     */
    usersProblemCreate = (data: ProblemUser, params: RequestParams = {}) =>
        this.request<ProblemUser, any>({
            path: `/api/users-problem/`,
            method: "POST",
            body: data,
            secure: true,
            type: ContentType.Json,
            format: "json",
            ...params,
        });
    /**
     * No description
     *
     * @tags users-problem
     * @name UsersProblemRetrieve
     * @summary Детальная информация о задачах пользователя
     * @request GET:/api/users-problem/{id}/
     * @secure
     */
    usersProblemRetrieve = (id: number, params: RequestParams = {}) =>
        this.request<ProblemUser, any>({
            path: `/api/users-problem/${id}/`,
            method: "GET",
            secure: true,
            format: "json",
            ...params,
        });
    /**
     * No description
     *
     * @tags users-problem
     * @name UsersProblemUpdate
     * @summary Обновление данных о задачах пользователя
     * @request PUT:/api/users-problem/{id}/
     * @secure
     */
    usersProblemUpdate = (id: number, data: ProblemUser, params: RequestParams = {}) =>
        this.request<ProblemUser, any>({
            path: `/api/users-problem/${id}/`,
            method: "PUT",
            body: data,
            secure: true,
            type: ContentType.Json,
            format: "json",
            ...params,
        });
    /**
     * No description
     *
     * @tags users-problem
     * @name UsersProblemPartialUpdate
     * @summary Обновление с необ. полями задачах пользователю
     * @request PATCH:/api/users-problem/{id}/
     * @secure
     */
    usersProblemPartialUpdate = (id: number, data: PatchedProblemUser, params: RequestParams = {}) =>
        this.request<ProblemUser, any>({
            path: `/api/users-problem/${id}/`,
            method: "PATCH",
            body: data,
            secure: true,
            type: ContentType.Json,
            format: "json",
            ...params,
        });
    /**
     * No description
     *
     * @tags users-problem
     * @name UsersProblemDestroy
     * @summary Удаление задачах пользователю
     * @request DELETE:/api/users-problem/{id}/
     * @secure
     */
    usersProblemDestroy = (id: number, params: RequestParams = {}) =>
        this.request<ProblemUser, any>({
            path: `/api/users-problem/${id}/`,
            method: "DELETE",
            secure: true,
            format: "json",
            ...params,
        });
}

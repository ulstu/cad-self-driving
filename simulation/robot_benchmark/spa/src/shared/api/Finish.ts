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

import { HttpClient, RequestParams } from "./http-client";

export class Finish<SecurityDataType = unknown> extends HttpClient<SecurityDataType> {
    /**
     * @description Удаляет докер контейнер и файлы в ОС пользователя по айди записи в таблице ProblemUser
     *
     * @tags finish
     * @name FinishRetrieve
     * @request GET:/api/finish/{problemuser_id}
     * @secure
     */
    finishRetrieve = (problemuserId: number, params: RequestParams = {}) =>
        this.request<void, any>({
            path: `/api/finish/${problemuserId}`,
            method: "GET",
            secure: true,
            ...params,
        });
}

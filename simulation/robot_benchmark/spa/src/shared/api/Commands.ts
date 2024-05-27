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

export class Commands<SecurityDataType = unknown> extends HttpClient<SecurityDataType> {
    /**
     * @description Исполняет первую команду в очереди и удаляет ее из очереди
     *
     * @tags commands
     * @name CommandsRetrieve
     * @request GET:/api/commands/
     * @secure
     */
    commandsRetrieve = (params: RequestParams = {}) =>
        this.request<void, any>({
            path: `/api/commands/`,
            method: "GET",
            secure: true,
            ...params,
        });
}

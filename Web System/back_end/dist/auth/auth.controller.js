"use strict";
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.authController = void 0;
const auth_service_1 = require("./auth.service");
function auth(req, res) {
    return __awaiter(this, void 0, void 0, function* () {
        const { email, password } = req.body;
        try {
            const token = yield auth_service_1.authService.auth(email, password);
            if (token) {
                res.json({
                    token: token
                });
            }
            else {
                res.status(401).json({
                    message: 'Email ou senha invalidos',
                });
            }
        }
        catch (error) {
            res.json({
                message: 'Error no servidor',
                error
            });
        }
    });
}
function authRegister(req, res) {
    return __awaiter(this, void 0, void 0, function* () {
        const { email, name, password } = req.body;
        try {
            const user = yield auth_service_1.authService.authRegister(email, name, password);
            res.status(201).json({
                message: 'Sucesso no registro',
                user
            });
        }
        catch (error) {
            res.json({
                message: 'Error no servidor',
                error
            });
        }
    });
}
exports.authController = {
    auth,
    authRegister
};

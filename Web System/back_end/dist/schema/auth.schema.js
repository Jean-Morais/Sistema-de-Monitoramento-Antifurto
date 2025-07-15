"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.authRegisterSchema = exports.authSchema = void 0;
const zod_1 = require("zod");
exports.authSchema = zod_1.z.object({
    email: zod_1.z.string().email('E-mail inválido'),
    password: zod_1.z.string().min(1, 'Senha é obrigatório')
});
exports.authRegisterSchema = zod_1.z.object({
    name: zod_1.z.string().min(1, 'Nome é obrigatório'),
    email: zod_1.z.string().email('E-mail inválido'),
    password: zod_1.z.string()
        .min(8, { message: 'A senha deve ter pelo menos 8 caracteres' })
        .regex(/[a-z]/, { message: 'A senha deve conter pelo menos uma letra minúscula' })
        .regex(/[A-Z]/, { message: 'A senha deve conter pelo menos uma letra maiúscula' })
        .regex(/[0-9]/, { message: 'A senha deve conter pelo menos um número' })
        .regex(/[^A-Za-z0-9]/, { message: 'A senha deve conter pelo menos um caractere especial' }),
});

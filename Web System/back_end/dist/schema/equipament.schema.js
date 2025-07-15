"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.querySchema = exports.findByIdSchema = exports.alertOutRangeSchema = exports.createEquipmentSchema = void 0;
const zod_1 = require("zod");
exports.createEquipmentSchema = zod_1.z.object({
    name: zod_1.z.string().min(1, 'Nome é obrigatório'),
    type: zod_1.z.string().min(1, 'Tipo é obrigatório'),
    location: zod_1.z.string().min(1, 'Localização é obrigatória'),
    rfid: zod_1.z.string().min(1, 'RFID é obrigatório'),
    responsible: zod_1.z.string().optional().or(zod_1.z.literal('')), // permite string vazia também
});
exports.alertOutRangeSchema = zod_1.z.object({
    rfid: zod_1.z.string().min(1, 'RFID é obrigatório')
});
exports.findByIdSchema = zod_1.z.object({
    id: zod_1.z.string()
        .transform((val) => Number(val)) // transforma em número
        .refine((val) => !isNaN(val), { message: 'ID inválido: deve ser um número' }),
});
exports.querySchema = zod_1.z.object({
    whereName: zod_1.z.string().optional(),
    type: zod_1.z.string().optional(),
    location: zod_1.z.string().optional(),
    responsible: zod_1.z.string().optional(),
    inRange: zod_1.z
        .union([zod_1.z.literal('true'), zod_1.z.literal('false')]) // porque query sempre vem como string
        .transform((val) => val === 'true')
        .optional(),
    page: zod_1.z
        .string()
        .optional()
        .transform((val) => (val ? parseInt(val, 10) : 1))
        .refine((val) => Number.isInteger(val) && val > 0, {
        message: 'page deve ser um número inteiro positivo',
    }),
    limit: zod_1.z
        .string()
        .optional()
        .transform((val) => (val ? parseInt(val, 10) : 10))
        .refine((val) => Number.isInteger(val) && val > 0, {
        message: 'limit deve ser um número inteiro positivo',
    }),
});

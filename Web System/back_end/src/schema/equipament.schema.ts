import { z } from 'zod';

export const createEquipmentSchema = z.object({
  name: z.string().min(1, 'Nome é obrigatório'),
  type: z.string().min(1, 'Tipo é obrigatório'),
  location: z.string().min(1, 'Localização é obrigatória'),
  rfid: z.string().min(1, 'RFID é obrigatório'),
  responsible: z.string().optional().or(z.literal('')), // permite string vazia também
});

export const alertOutRangeSchema = z.object({
  rfid: z.string().min(1, 'RFID é obrigatório')
});

export const findByIdSchema = z.object({
  id: z.string()
    .transform((val) => Number(val)) // transforma em número
    .refine((val) => !isNaN(val), { message: 'ID inválido: deve ser um número' }),
});

export const updateByRFIDParam = z.object({
  rfid: z.string().min(1, 'RFID valido é obrigatório')
});

export const updateByRFIDBody = z.object({
  name: z.string().min(1, 'Nome é obrigatório'),
  type: z.string().min(1, 'Tipo é obrigatório'),
  location: z.string().min(1, 'Localização é obrigatória'),
  responsible: z.string().optional().or(z.literal('')), 
})

export const deleteByRFIDParam = z.object({
  rfid: z.string().min(1, 'RFID valido é obrigatório')
});

export const changeStatusParam = z.object({
  rfid: z.string().min(1, 'RFID valido é obrigatório')
});

export const querySchema = z.object({
  whereName: z.string().optional(),
  type: z.string().optional(),
  location: z.string().optional(),
  responsible: z.string().optional(),
  inRange: z
    .union([z.literal('true'), z.literal('false')]) // porque query sempre vem como string
    .transform((val) => val === 'true')
    .optional(),
  page: z
    .string()
    .optional()
    .transform((val) => (val ? parseInt(val, 10) : 1))
    .refine((val) => Number.isInteger(val) && val > 0, {
      message: 'page deve ser um número inteiro positivo',
    }),

  limit: z
    .string()
    .optional()
    .transform((val) => (val ? parseInt(val, 10) : 10))
    .refine((val) => Number.isInteger(val) && val > 0, {
      message: 'limit deve ser um número inteiro positivo',
    }),
});

export type EquipmentFilters = z.infer<typeof querySchema>;